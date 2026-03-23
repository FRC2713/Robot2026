package frc2713.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2713.lib.io.AdvantageScopePathBuilder;
import frc2713.lib.logging.PeriodicTimingLogger;
import frc2713.lib.logging.TimeLogged;
import frc2713.lib.subsystem.KinematicsManager;
import frc2713.lib.util.AllianceFlipUtil;
import frc2713.lib.util.LoggedTunableBoolean;
import frc2713.robot.FieldConstants;
import frc2713.robot.RobotContainer;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

public class LaunchingSolutionManager extends SubsystemBase {
  private static LaunchingSolutionManager instance;

  private static Rotation2d manualOffset = new Rotation2d(LauncherConstants.Turret.manualOffset);

  AdvantageScopePathBuilder pb = new AdvantageScopePathBuilder("LaunchingSolutionManager");

  /**
   * When true (via {@code /Tuning/...} in tuning mode), {@link #calculateITOF} logs extra geometry
   * and iteration data under {@code LaunchingSolutionManager/itof debug/...}.
   */
  public static final LoggedTunableBoolean itofDebug =
      new LoggedTunableBoolean("LaunchingSolutionManager/itof_debug", false);

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

  public static Translation3d currentGoal = FieldConstants.Hub.target.getCenter3d();
  public static Distance targetRadius = Meters.of(FieldConstants.Hub.target.getRadiusMeters());

  private static InterpolatingDoubleTreeMap currentHoodMap =
      LaunchingLookupMaps.distanceToAngleMap; // use for dist -> hood angle
  private static InterpolatingDoubleTreeMap currentRPMMap =
      LaunchingLookupMaps.distanceToRpmMap; // use for dist -> flywheel rpm
  private static InterpolatingDoubleTreeMap currentTofMap = LaunchingLookupMaps.tofMap;

  /** Warm start for iterative ToF (seconds). */
  private double lastItofTofSeconds = -1.0;

  private final SendableChooser<LauncherConstants.LaunchSolverMode> launchSolverModeChooser =
      new SendableChooser<>();

  public LaunchingSolutionManager() {
    if (instance != null) {
      throw new IllegalStateException("LaunchingSolutionManager already initialized!");
    }
    instance = this;

    launchSolverModeChooser.addOption("Static (LUT)", LauncherConstants.LaunchSolverMode.STATIC);
    launchSolverModeChooser.addOption(
        "Vector approx", LauncherConstants.LaunchSolverMode.VECTOR_APPROX);
    launchSolverModeChooser.setDefaultOption("IToF", LauncherConstants.LaunchSolverMode.ITOF);
    SmartDashboard.putData("Launch Solver Mode", launchSolverModeChooser);
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

      // 3. Solve for the Launch Vector
      LauncherConstants.LaunchSolverMode solverMode = launchSolverModeChooser.getSelected();
      if (solverMode == null) {
        solverMode = LauncherConstants.LaunchSolverMode.STATIC;
      }
      currentSolution =
          switch (solverMode) {
            case VECTOR_APPROX -> calculateVectorApprox(
                robotPose, robotLinVel, LaunchingSolutionManager.currentGoal);
            case ITOF -> calculateITOF(
                robotPose, robotLinVel, LaunchingSolutionManager.currentGoal);
            case STATIC -> calculateStatic(
                robotPose, robotLinVel, LaunchingSolutionManager.currentGoal);
          };

      Logger.recordOutput(pb.makePath("solver mode"), solverMode.name());

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

    // Normalized horizontal direction to goal
    Translation3d horizontalDir =
        new Translation3d(rangeVec.getX(), rangeVec.getY(), 0)
            .div(rangeVec.toTranslation2d().getNorm());

    // Get Horizontal Angle (Yaw) This is a field-relative angle, NOT robot-relative.
    // The conversion to robot-relative happens at the point of use (e.g. in Turret)
    // so that it always uses the robot's current heading, not a stale or projected one.
    Rotation2d newYaw =
        new Rotation2d(Math.atan2(horizontalDir.getY(), horizontalDir.getX()))
            .rotateBy(manualOffset);

    return new LaunchSolution(
        newYaw,
        LaunchingSolutionManager.currentRPMMap.get(horizontalDist),
        Rotation2d.fromDegrees(LaunchingSolutionManager.currentHoodMap.get(horizontalDist)),
        horizontalDist,
        true);
  }

  /**
   * Vector approximation OTF: keep the same ideal field-relative launch vector as {@link
   * #calculateStatic} (LUT speed + hood), then subtract robot velocity and derive flywheel RPM from
   * the resulting muzzle speed and hood pitch from the resulting vector.
   */
  private LaunchSolution calculateVectorApprox(
      Pose3d robotPose, Translation3d robotVel, Translation3d targetPose) {
    // A. Relative Position
    Translation3d rangeVec = targetPose.minus(robotPose.getTranslation());
    // Use 2D horizontal distance for map lookups (maps are indexed by ground distance)
    double horizontalDist = rangeVec.toTranslation2d().getNorm();

    // B. Check Range
    // if (horizontalDist > 8.0 || horizontalDist < 0.1) {
    //   return new LaunchSolution(new Rotation2d(), 0, new Rotation2d(), horizontalDist, false);
    // }

    // C. LUT + kinematics → ideal field-frame launch vector
    double rpmSetpoint = LaunchingSolutionManager.currentRPMMap.get(horizontalDist);
    double idealSpeed = LaunchingLookupMaps.muzzleVelocityMetersPerSecond(rpmSetpoint);
    Logger.recordOutput(pb.makePath("ideal ball speed"), idealSpeed);
    double idealPitchRad =
        LaunchingLookupMaps.exitAngleRadiansFromHoodDegrees(
            LaunchingSolutionManager.currentHoodMap.get(horizontalDist));

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
    double newLauncherSpeed = LaunchingLookupMaps.velocityToRpmBiDiMap.get(newBallSpeed);

    // Vertical Angle (Pitch)
    double newPitch =
        Math.atan2(
            neededMuzzleVelocity.getZ(),
            Math.hypot(neededMuzzleVelocity.getX(), neededMuzzleVelocity.getY()));

    // Horizontal Angle (Yaw) - this is a field-relative angle, NOT robot-relative.
    // The conversion to robot-relative happens at the point of use (e.g. in Turret)
    // so that it always uses the robot's current heading, not a stale or projected one.
    Rotation2d newYaw =
        new Rotation2d(Math.atan2(neededMuzzleVelocity.getY(), neededMuzzleVelocity.getX()))
            .rotateBy(manualOffset);

    return new LaunchSolution(
        newYaw, newLauncherSpeed, new Rotation2d(newPitch), horizontalDist, true);
  }

  /**
   * Iterative time-of-flight: self-consistent t where drag-aware ToF from {@link
   * LaunchingLookupMaps#tofMap} matches projected horizontal range while the robot translates at
   * constant velocity.
   */
  private LaunchSolution calculateITOF(
      Pose3d robotPose, Translation3d robotVel, Translation3d targetPose) {
    boolean dbg = itofDebug.get();
    Translation3d robotTrans = robotPose.getTranslation();
    double tMin = LauncherConstants.itofTofMin.get().in(Seconds);
    double tMax = LauncherConstants.itofTofMax.get().in(Seconds);
    double eps = LauncherConstants.itofConvergenceSeconds.get().in(Seconds);
    int maxIter = (int) Math.round(LauncherConstants.itofMaxIterations.get());

    Translation3d rangeVec0 = targetPose.minus(robotTrans);
    double d0 = rangeVec0.toTranslation2d().getNorm();
    if (d0 < 0.05) {
      if (dbg) {
        Logger.recordOutput(pb.makePath("itof debug", "status"), "fallback d0 too small");
      }
      lastItofTofSeconds = -1.0;
      return calculateStatic(robotPose, robotVel, targetPose);
    }

    double t = lastItofTofSeconds;
    if (t < tMin || t > tMax || Double.isNaN(t)) {
      t = MathUtil.clamp(LaunchingSolutionManager.currentTofMap.get(d0), tMin, tMax);
    }

    if (dbg) {
      Logger.recordOutput(pb.makePath("itof debug", "rangeVec0"), rangeVec0);
      Logger.recordOutput(pb.makePath("itof debug", "d0 m"), d0);
      Logger.recordOutput(pb.makePath("itof debug", "robot translation"), robotTrans);
      Logger.recordOutput(pb.makePath("itof debug", "robot pose"), robotPose);
      Logger.recordOutput(pb.makePath("itof debug", "robot lin vel"), robotVel);
      Logger.recordOutput(pb.makePath("itof debug", "target"), targetPose);
      Logger.recordOutput(pb.makePath("itof debug", "t init s"), t);
      Logger.recordOutput(
          pb.makePath("itof debug", "tof from d0 s"),
          LaunchingSolutionManager.currentTofMap.get(d0));
      Logger.recordOutput(pb.makePath("itof debug", "t min s"), tMin);
      Logger.recordOutput(pb.makePath("itof debug", "t max s"), tMax);
      Logger.recordOutput(pb.makePath("itof debug", "convergence eps s"), eps);
      Logger.recordOutput(pb.makePath("itof debug", "max iterations"), maxIter);
    }

    double horizontalDist = d0;
    Translation3d rel = rangeVec0;
    int iterationsUsed = 0;

    double[] tPerIter = dbg && maxIter > 0 ? new double[maxIter] : null;
    double[] dPerIter = dbg && maxIter > 0 ? new double[maxIter] : null;
    double[] tNextPerIter = dbg && maxIter > 0 ? new double[maxIter] : null;

    for (int i = 0; i < maxIter; i++) {
      Translation3d projected =
          robotTrans.plus(
              new Translation3d(robotVel.getX() * t, robotVel.getY() * t, robotVel.getZ() * t));
      rel = targetPose.minus(projected);
      horizontalDist = rel.toTranslation2d().getNorm();
      if (horizontalDist < 0.05) {
        if (dbg) {
          Logger.recordOutput(pb.makePath("itof debug", "status"), "fallback iter d_h too small");
          Logger.recordOutput(pb.makePath("itof debug", "last iter"), i);
          logItofDebugIteration(
              i, t, projected, rel, horizontalDist, Double.NaN, robotPose.getRotation());
        }
        lastItofTofSeconds = -1.0;
        return calculateStatic(robotPose, robotVel, targetPose);
      }

      double tNext =
          MathUtil.clamp(LaunchingSolutionManager.currentTofMap.get(horizontalDist), tMin, tMax);
      iterationsUsed = i + 1;
      if (tPerIter != null) {
        tPerIter[i] = t;
        dPerIter[i] = horizontalDist;
        tNextPerIter[i] = tNext;
      }
      if (dbg) {
        logItofDebugIteration(i, t, projected, rel, horizontalDist, tNext, robotPose.getRotation());
      }
      if (Math.abs(tNext - t) < eps) {
        t = tNext;
        break;
      }
      t = tNext;
    }

    lastItofTofSeconds = t;
    Logger.recordOutput(pb.makePath("itof iterations"), iterationsUsed);
    Logger.recordOutput(pb.makePath("itof tof s"), t);

    if (dbg && tPerIter != null && iterationsUsed > 0) {
      Logger.recordOutput(
          pb.makePath("itof debug", "t per iter"), Arrays.copyOf(tPerIter, iterationsUsed));
      Logger.recordOutput(
          pb.makePath("itof debug", "d_h per iter m"), Arrays.copyOf(dPerIter, iterationsUsed));
      Logger.recordOutput(
          pb.makePath("itof debug", "t next per iter s"),
          Arrays.copyOf(tNextPerIter, iterationsUsed));
    }

    Translation3d projected =
        robotTrans.plus(
            new Translation3d(robotVel.getX() * t, robotVel.getY() * t, robotVel.getZ() * t));
    rel = targetPose.minus(projected);
    horizontalDist = rel.toTranslation2d().getNorm();

    if (dbg) {
      Logger.recordOutput(
          pb.makePath("itof debug", "projected robot pose"),
          new Pose3d(projected, robotPose.getRotation()));
      Logger.recordOutput(pb.makePath("itof debug", "final rel to target"), rel);
      Logger.recordOutput(pb.makePath("itof debug", "final d_h m"), horizontalDist);
      Logger.recordOutput(pb.makePath("itof debug", "converged t s"), t);
    }

    if (horizontalDist < 0.05 || Double.isNaN(horizontalDist)) {
      if (dbg) {
        Logger.recordOutput(pb.makePath("itof debug", "status"), "fallback final d_h invalid");
      }
      lastItofTofSeconds = -1.0;
      return calculateStatic(robotPose, robotVel, targetPose);
    }

    double rpmSetpoint = LaunchingSolutionManager.currentRPMMap.get(horizontalDist);
    double hoodDeg = LaunchingSolutionManager.currentHoodMap.get(horizontalDist);
    double idealBallSpeed = LaunchingLookupMaps.muzzleVelocityMetersPerSecond(rpmSetpoint);
    double idealPitchRad = LaunchingLookupMaps.exitAngleRadiansFromHoodDegrees(hoodDeg);

    Translation3d horizontalDir = new Translation3d(rel.getX(), rel.getY(), 0).div(horizontalDist);

    Translation3d idealVelocity =
        horizontalDir
            .times(idealBallSpeed * Math.cos(idealPitchRad))
            .plus(new Translation3d(0, 0, idealBallSpeed * Math.sin(idealPitchRad)));

    Translation3d neededMuzzleVelocity = idealVelocity.minus(robotVel);

    if (dbg) {
      Logger.recordOutput(pb.makePath("itof debug", "rpm lut"), rpmSetpoint);
      Logger.recordOutput(pb.makePath("itof debug", "hood deg lut"), hoodDeg);
      Logger.recordOutput(pb.makePath("itof debug", "ideal ball speed mps"), idealBallSpeed);
      Logger.recordOutput(pb.makePath("itof debug", "ideal pitch rad"), idealPitchRad);
      Logger.recordOutput(pb.makePath("itof debug", "ideal velocity"), idealVelocity);
      Logger.recordOutput(
          pb.makePath("itof debug", "needed muzzle velocity"), neededMuzzleVelocity);
      Logger.recordOutput(pb.makePath("itof debug", "status"), "ok");
    }

    Rotation2d newYaw =
        new Rotation2d(Math.atan2(neededMuzzleVelocity.getY(), neededMuzzleVelocity.getX()))
            .rotateBy(manualOffset);

    return new LaunchSolution(
        newYaw, rpmSetpoint, Rotation2d.fromDegrees(hoodDeg), horizontalDist, true);
  }

  private void logItofDebugIteration(
      int i,
      double t,
      Translation3d projectedTrans,
      Translation3d relToTarget,
      double dHorizontal,
      double tNext,
      Rotation3d robotRotation) {
    Logger.recordOutput(pb.makePath("itof debug", "iter index"), i);
    Logger.recordOutput(pb.makePath("itof debug", "iter t s"), t);
    Logger.recordOutput(
        pb.makePath("itof debug", "iter projected pose"),
        new Pose3d(projectedTrans, robotRotation));
    Logger.recordOutput(pb.makePath("itof debug", "iter rel to target"), relToTarget);
    Logger.recordOutput(pb.makePath("itof debug", "iter d_h m"), dHorizontal);
    Logger.recordOutput(pb.makePath("itof debug", "iter t next s"), tNext);
  }

  public class ZoneSelectionHelpers {

    public static void configureForFeeding(Pose2d robotPose) {
      boolean bottom = robotPose.getTranslation().getY() < FieldConstants.LinesHorizontal.center;

      if (bottom) {

        LaunchingSolutionManager.currentGoal =
            AllianceFlipUtil.applyX(FieldConstants.AllianceZone.bottomPassingTarget.getCenter3d());

        LaunchingSolutionManager.targetRadius =
            Meters.of(FieldConstants.AllianceZone.bottomPassingTarget.getRadiusMeters());
      } else {
        LaunchingSolutionManager.currentGoal =
            AllianceFlipUtil.applyX(FieldConstants.AllianceZone.topPassingTarget.getCenter3d());

        LaunchingSolutionManager.targetRadius =
            Meters.of(FieldConstants.AllianceZone.topPassingTarget.getRadiusMeters());
      }
      LaunchingSolutionManager.currentHoodMap = LaunchingLookupMaps.distanceToAngleAzMap;
      LaunchingSolutionManager.currentRPMMap = LaunchingLookupMaps.distanceToRpmAzMap;
      LaunchingSolutionManager.currentTofMap = LaunchingLookupMaps.tofMapAZ;
    }

    public static void configureForScoring() {
      LaunchingSolutionManager.currentGoal =
          AllianceFlipUtil.apply(FieldConstants.Hub.target.getCenter3d());
      LaunchingSolutionManager.currentHoodMap = LaunchingLookupMaps.distanceToAngleMap;
      LaunchingSolutionManager.currentRPMMap = LaunchingLookupMaps.distanceToRpmMap;
      LaunchingSolutionManager.currentTofMap = LaunchingLookupMaps.tofMap;
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
