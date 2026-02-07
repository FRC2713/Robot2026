package frc2713.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2713.lib.io.AdvantageScopePathBuilder;
import frc2713.lib.subsystem.KinematicsManager;
import frc2713.lib.util.AllianceFlipUtil;
import frc2713.robot.FieldConstants;
import org.littletonrobotics.junction.Logger;

public class LaunchingSolutionManager extends SubsystemBase {
  private static LaunchingSolutionManager instance;

  AdvantageScopePathBuilder pb = new AdvantageScopePathBuilder("LaunchingSolutionManager");

  // --- Data Structures ---
  public static record LaunchSolution(
      Rotation2d turretFieldRelativeYaw, // desired global yaw for the turret
      double flywheelSpeedMetersPerSecond, // Required exit velocity
      Rotation2d hoodPitch, // Required vertical angle
      double effectiveDistanceMeters,
      boolean isValid // False if target is out of range or blocked
      ) {}

  public static record FieldGoal(Translation3d flywheelTarget, Translation3d positionalTarget) {}

  // Default to an empty/invalid solution
  private LaunchSolution currentSolution =
      new LaunchSolution(new Rotation2d(), 0, new Rotation2d(), 0, false);

  public LaunchingSolutionManager() {
    if (instance != null) {
      throw new IllegalStateException("LaunchingSolutionManager already initialized!");
    }
    instance = this;
  }

  public static FieldGoal currentGoal = new FieldGoal(FieldConstants.Hub.innerCenterPoint, FieldConstants.Hub.topCenterPoint);
  
  public static void setFieldGoal(Translation3d flywheelTarget, Translation3d positionalTarget) {
    LaunchingSolutionManager.currentGoal =
        new FieldGoal(
            AllianceFlipUtil.apply(flywheelTarget), AllianceFlipUtil.apply(positionalTarget));
  }

  public static LaunchingSolutionManager getInstance() {
    // Lazy load logic could go here, but for Subsystems, constructor is better
    return instance;
  }

  @Override
  public void periodic() {
    // 1. Get Robot State (ID 0 = Chassis)
    Pose3d robotPose = KinematicsManager.getInstance().getGlobalPose(0);
    Translation3d robotLinVel = KinematicsManager.getInstance().getGlobalLinearVelocity(0);

    // 2. Solve for the Launch Vector
    if (LauncherConstants.otfFutureProjectionEnabled.get()) {
      Rotation3d robotAngVel = KinematicsManager.getInstance().getGlobalAngularVelocity(0);
      currentSolution =
          calculate(
              robotPose,
              robotLinVel,
              robotAngVel,
              LaunchingSolutionManager.currentGoal.positionalTarget);
    } else {
      currentSolution =
          calculate(robotPose, robotLinVel, LaunchingSolutionManager.currentGoal.positionalTarget);
    }
  }

  public LaunchSolution getSolution() {
    return currentSolution;
  }

  private LaunchSolution calculate(
      Pose3d robotPose, Translation3d linearVel, Rotation3d angularVel, Translation3d targetPose) {
    Time timeToProject = LauncherConstants.otfFutureProjectionSeconds.get();
    Pose3d projectedPose =
        KinematicsManager.getInstance()
            .limitPoseToField(
                new Pose3d(
                    robotPose.getTranslation().plus(linearVel.times(timeToProject.in(Seconds))),
                    robotPose.getRotation().plus(angularVel.times(timeToProject.in(Seconds)))));
    Logger.recordOutput(pb.makePath("projected_pose"), projectedPose);
    return calculate(robotPose, linearVel, targetPose);
  }

  private LaunchSolution calculate(
      Pose3d robotPose, Translation3d robotVel, Translation3d targetPos) {
    // A. Relative Position
    Translation3d rangeVec = targetPos.minus(robotPose.getTranslation());
    double dist = rangeVec.getNorm();

    // B. Check Range
    if (dist > 8.0 || dist < 1.0) {
      return new LaunchSolution(new Rotation2d(), 0, new Rotation2d(), dist, false);
    }

    // C. Get Ideal Static Launch Params (Ground Relative)
    double idealSpeed =
        FeetPerSecond.of(LauncherConstants.Flywheels.velocityMap.get(dist)).in(MetersPerSecond);
    double idealPitchRad = Math.toRadians(LauncherConstants.Hood.angleMap.get(dist));

    // D. Construct Ideal Velocity Vector
    // Normalized horizontal direction to goal
    Translation3d horizontalDir =
        new Translation3d(rangeVec.getX(), rangeVec.getY(), 0)
            .div(rangeVec.toTranslation2d().getNorm());

    // Combine Horizontal and Vertical components
    Translation3d idealVelocity =
        horizontalDir
            .times(idealSpeed * Math.cos(idealPitchRad))
            .plus(new Translation3d(0, 0, idealSpeed * Math.sin(idealPitchRad)));

    // E. Subtract Robot Velocity (V_muzzle = V_ideal - V_robot)
    Translation3d neededMuzzleVelocity = idealVelocity.minus(robotVel);

    // F. Extract Parameters from Resulting Vector
    double newSpeed = neededMuzzleVelocity.getNorm();

    // Vertical Angle (Pitch)
    double newPitch =
        Math.atan2(
            neededMuzzleVelocity.getZ(),
            Math.hypot(neededMuzzleVelocity.getX(), neededMuzzleVelocity.getY()));

    // Horizontal Angle (Yaw)
    double newYaw = Math.atan2(neededMuzzleVelocity.getY(), neededMuzzleVelocity.getX());

    return new LaunchSolution(
        new Rotation2d(newYaw), newSpeed, new Rotation2d(newPitch), dist, true);
  }
}
