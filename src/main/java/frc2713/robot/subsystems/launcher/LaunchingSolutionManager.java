package frc2713.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2713.lib.io.AdvantageScopePathBuilder;
import frc2713.lib.logging.PeriodicTimingLogger;
import frc2713.lib.logging.TimeLogged;
import frc2713.lib.subsystem.KinematicsManager;
import frc2713.lib.util.AllianceFlipUtil;
import frc2713.robot.FieldConstants;
import frc2713.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class LaunchingSolutionManager extends SubsystemBase {
  private static LaunchingSolutionManager instance;

  AdvantageScopePathBuilder pb = new AdvantageScopePathBuilder("LaunchingSolutionManager");

  // --- Data Structures ---
  public static record LaunchSolution(
      Rotation2d turretFieldYaw, // desired global yaw for the turret
      double flywheelsRPM, // desired velocity setpoint for flywheels
      Rotation2d hoodPitch, // Required vertical angle
      double effectiveDistanceMeters,
      boolean isValid // False if target is out of range or blocked
      ) {}

  // Default to an empty/invalid solution
  private LaunchSolution currentSolution =
      new LaunchSolution(new Rotation2d(), 0, new Rotation2d(), 0, false);

  public static Translation3d currentGoal = FieldConstants.Hub.topCenterPoint;
  private static InterpolatingDoubleTreeMap currentHoodMap =
      LauncherConstants.Hood.angleMap; // use for dist -> hood angle
  private static InterpolatingDoubleTreeMap currentBallSpeedMap =
      LauncherConstants.Flywheels.ballVelocityMap; // use for dist -> ball velocity
  private static InterpolatingDoubleTreeMap currentRPMMap =
      LauncherConstants.Flywheels.rpmVelocityMap; // use for dist -> flywheel rpm

  public LaunchingSolutionManager() {
    if (instance != null) {
      throw new IllegalStateException("LaunchingSolutionManager already initialized!");
    }
    instance = this;
  }

  public static LaunchingSolutionManager getInstance() {
    // Lazy load logic could go here, but for Subsystems, constructor is better
    return instance;
  }

  @Override
  @TimeLogged("Performance/SubsystemPeriodic/LaunchingSolutionManager")
  public void periodic() {
    try (var ignored = PeriodicTimingLogger.time(this)) {
      // 1. Get Robot State (ID 0 = Chassis)
      Pose3d robotPose = KinematicsManager.getInstance().getGlobalPose(0);
      Translation3d robotLinVel = KinematicsManager.getInstance().getGlobalLinearVelocity(0);

      // 2. Select goal
      if (FieldConstants.NeutralZone.region.contains(
          robotPose.getTranslation().toTranslation2d())) {
        ZoneSelectionHelpers.configureForFeeding(robotPose.toPose2d());
        Logger.recordOutput(pb.makePath("shot selection"), "Feeding");
      } else {
        ZoneSelectionHelpers.configureForScoring();
        Logger.recordOutput(pb.makePath("shot selection"), "Scoring");
      }

      // 3. Solve for the Launch Vector
      currentSolution =
          calculateStatic(robotPose, robotLinVel, LaunchingSolutionManager.currentGoal);

      // 4. Log
      Logger.recordOutput(pb.makePath("used robot pose"), robotPose);
      Logger.recordOutput(pb.makePath("used robot lin vel"), robotLinVel);
      Logger.recordOutput(pb.makePath("current goal"), LaunchingSolutionManager.currentGoal);
      Logger.recordOutput(pb.makePath("current solution"), currentSolution);
    }
  }

  public LaunchSolution getSolution() {
    return currentSolution;
  }

  private LaunchSolution calculateStatic(
      Pose3d robotPose, Translation3d robotVel, Translation3d targetPose) {
    // A. Relative Position
    Translation3d rangeVec = targetPose.minus(robotPose.getTranslation());
    // Use 2D horizontal distance for map lookups (maps are indexed by ground distance)
    double horizontalDist = rangeVec.toTranslation2d().getNorm();

    // B. Check Range
    // if (horizontalDist > 8.0 || horizontalDist < 0.1) {
    //   return new LaunchSolution(new Rotation2d(), 0, new Rotation2d(), horizontalDist, false);
    // }

    // C. All the math still needed to calculate turret angle
    // Get Ideal Static Launch Params (Ground Relative)
    double idealBallSpeed =
        FeetPerSecond.of(LaunchingSolutionManager.currentBallSpeedMap.get(horizontalDist))
            .in(MetersPerSecond);
    double idealPitchRad =
        Math.toRadians(LaunchingSolutionManager.currentHoodMap.get(horizontalDist));

    // Normalized horizontal direction to goal
    Translation3d horizontalDir =
        new Translation3d(rangeVec.getX(), rangeVec.getY(), 0)
            .div(rangeVec.toTranslation2d().getNorm());

    // Combine Horizontal and Vertical components
    Translation3d idealVelocity =
        horizontalDir
            .times(idealBallSpeed * Math.cos(idealPitchRad))
            .plus(new Translation3d(0, 0, idealBallSpeed * Math.sin(idealPitchRad)));

    // Subtract Robot Velocity (V_muzzle = V_ideal - V_robot)
    Translation3d neededMuzzleVelocity = idealVelocity.minus(robotVel);

    // Get Horizontal Angle (Yaw) This is a field-relative angle, NOT robot-relative.
    // The conversion to robot-relative happens at the point of use (e.g. in Turret)
    // so that it always uses the robot's current heading, not a stale or projected one.
    Rotation2d newYaw =
        new Rotation2d(Math.atan2(neededMuzzleVelocity.getY(), neededMuzzleVelocity.getX()))
            .rotateBy(new Rotation2d(Math.PI));

    return new LaunchSolution(
        newYaw,
        LaunchingSolutionManager.currentRPMMap.get(horizontalDist),
        Rotation2d.fromDegrees(LaunchingSolutionManager.currentHoodMap.get(horizontalDist)),
        horizontalDist,
        true);
  }

  private LaunchSolution calculateOTF(
      Pose3d robotPose, Translation3d robotVel, Translation3d targetPose) {
    // A. Relative Position
    Translation3d rangeVec = targetPose.minus(robotPose.getTranslation());
    // Use 2D horizontal distance for map lookups (maps are indexed by ground distance)
    double horizontalDist = rangeVec.toTranslation2d().getNorm();

    // B. Check Range
    // if (horizontalDist > 8.0 || horizontalDist < 0.1) {
    //   return new LaunchSolution(new Rotation2d(), 0, new Rotation2d(), horizontalDist, false);
    // }

    // C. Get Ideal Static Launch Params (Ground Relative)
    double idealSpeed =
        FeetPerSecond.of(LaunchingSolutionManager.currentBallSpeedMap.get(horizontalDist))
            .in(MetersPerSecond);
    double idealPitchRad =
        Math.toRadians(LaunchingSolutionManager.currentHoodMap.get(horizontalDist));

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
    double newBallSpeed = neededMuzzleVelocity.getNorm();
    double newLauncherSpeed = LauncherConstants.Flywheels.ballToFlywheelMap.get(newBallSpeed);

    // Vertical Angle (Pitch)
    double newPitch =
        Math.atan2(
            neededMuzzleVelocity.getZ(),
            Math.hypot(neededMuzzleVelocity.getX(), neededMuzzleVelocity.getY()));

    // Horizontal Angle (Yaw) - this is a field-relative angle, NOT robot-relative.
    // The conversion to robot-relative happens at the point of use (e.g. in Turret)
    // so that it always uses the robot's current heading, not a stale or projected one.
    Rotation2d newYaw =
        new Rotation2d(Math.atan2(neededMuzzleVelocity.getY(), neededMuzzleVelocity.getX()));

    return new LaunchSolution(
        newYaw, newLauncherSpeed, new Rotation2d(newPitch), horizontalDist, true);
  }

  public class ZoneSelectionHelpers {

    public static void configureForFeeding(Pose2d robotPose) {
      LaunchingSolutionManager.currentGoal =
          robotPose.getTranslation().getY() < FieldConstants.LinesHorizontal.center
              ? AllianceFlipUtil.applyX(FieldConstants.AllianceZone.bottomSideCornerTarget)
              : AllianceFlipUtil.applyX(FieldConstants.AllianceZone.topSideCornerTarget);
      LaunchingSolutionManager.currentHoodMap = LauncherConstants.Hood.angleForAZMap;
      LaunchingSolutionManager.currentBallSpeedMap = LauncherConstants.Flywheels.ballVelocityAZMap;
      LaunchingSolutionManager.currentRPMMap = LauncherConstants.Flywheels.rpmVelocityAZMap;
    }

    public static void configureForScoring() {
      LaunchingSolutionManager.currentGoal =
          AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint);
      LaunchingSolutionManager.currentHoodMap = LauncherConstants.Hood.angleMap;
      LaunchingSolutionManager.currentBallSpeedMap = LauncherConstants.Flywheels.rpmVelocityMap;
      LaunchingSolutionManager.currentRPMMap = LauncherConstants.Flywheels.rpmVelocityMap;
    }

    public static Rotation2d storedIntakeRotation = new Rotation2d(0);

    public static void setIntakeRotation() {
      if (RobotContainer.drive.getPose().getTranslation().getX()
          < AllianceFlipUtil.applyX(FieldConstants.LinesVertical.allianceZone)) {
        storedIntakeRotation = new Rotation2d(0);
      } else if (RobotContainer.drive.getPose().getTranslation().getX()
          > AllianceFlipUtil.applyX(FieldConstants.LinesVertical.neutralZoneNear)) {
        storedIntakeRotation = new Rotation2d(Math.PI);
      } else {
        storedIntakeRotation = RobotContainer.drive.getRotation();
      }
    }
  }
}
