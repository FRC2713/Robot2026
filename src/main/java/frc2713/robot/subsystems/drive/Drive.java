// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc2713.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import choreo.trajectory.SwerveSample;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc2713.lib.io.AdvantageScopePathBuilder;
import frc2713.lib.io.ArticulatedComponent;
import frc2713.robot.Constants;
import frc2713.robot.Constants.Mode;
import frc2713.robot.FieldConstants;
import frc2713.robot.generated.TunerConstants;
import frc2713.robot.util.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase implements ArticulatedComponent {
  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY = TunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = 74.088;
  private static final double ROBOT_MOI = 6.883;
  private static final double WHEEL_COF = 1.2;
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              TunerConstants.FrontLeft.WheelRadius,
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              TunerConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);

  private final AdvantageScopePathBuilder odometryPb;
  private final AdvantageScopePathBuilder drivePb;

  // ----- Drive Limits -----
  // These limits let us cap how fast and how quickly the robot can change speed.
  // A value of POSITIVE_INFINITY means "no limit" (the default).
  private static final double LOOP_PERIOD_SECS = 0.02; // 20ms robot loop

  // Max allowed linear speed (m/s). Speeds above this get scaled down.
  private double linearVelocityLimit = Double.POSITIVE_INFINITY;
  // Max allowed linear acceleration (m/s^2). Limits how fast we speed up, slow down, or change
  // direction.
  private double linearAccelerationLimit = Double.POSITIVE_INFINITY;
  // Max allowed angular (turning) speed (rad/s). Rotation rates above this get clamped.
  private double angularVelocityLimit = Double.POSITIVE_INFINITY;
  // Max allowed angular acceleration (rad/s^2). Limits how fast the robot can start or stop
  // spinning.
  private double angularAccelerationLimit = Double.POSITIVE_INFINITY;

  // Remembers what we actually commanded last cycle so we can calculate acceleration
  private ChassisSpeeds lastCommandedSpeeds = new ChassisSpeeds();

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);
    this.drivePb = new AdvantageScopePathBuilder("Drive");
    this.odometryPb = new AdvantageScopePathBuilder("Odometry");

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(odometryPb.makePath("Trajectory"), activePath.toArray(new Pose2d[0]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput(odometryPb.makePath("TrajectorySetpoint"), targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput(drivePb.makePath("SysIdState"), state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public Transform3d getTransform3d() {
    Pose2d pos = this.poseEstimator.getEstimatedPosition();
    return new Transform3d(pos.getX(), pos.getY(), 0, new Rotation3d(pos.getRotation()));
  }

  @Override
  public Translation3d getRelativeLinearVelocity() {
    ChassisSpeeds speeds = getChassisSpeeds();

    return new Translation3d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0);
  }

  @Override
  public Translation3d getRelativeAngularVelocity() {
    ChassisSpeeds speeds = getChassisSpeeds();
    return new Translation3d(0, 0, speeds.omegaRadiansPerSecond);
  }

  @Override
  public Translation3d getRelativeLinearAcceleration() {
    // TODO: Implement acceleration tracking from gyro or differentiated velocity
    return new Translation3d();
  }

  @Override
  public Translation3d getRelativeAngularAcceleration() {
    // TODO: Implement angular acceleration tracking from gyro or differentiated velocity
    return new Translation3d();
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput(
          drivePb.makePath("SwerveStates", "Setpoints"), new SwerveModuleState[] {});
      Logger.recordOutput(
          drivePb.makePath("SwerveStates", "SetpointsOptimized"), new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
    Logger.recordOutput(
        "Drive/isInNeutralZone",
        FieldConstants.NeutralZone.region.contains(new Rectangle2d(getPose(), 1.0, 1.0)));
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Apply any configured velocity/acceleration limits before sending to modules
    speeds = applyDriveLimits(speeds);

    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput(drivePb.makePath("SwerveStates", "Setpoints"), setpointStates);
    Logger.recordOutput(drivePb.makePath("SwerveChassisSpeeds", "Setpoints"), discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput(drivePb.makePath("SwerveStates", "SetpointsOptimized"), setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  // ----- Drive Limit Setters -----

  /** Sets the maximum linear velocity (m/s). The robot won't drive faster than this. */
  public void setLinearVelocityLimit(LinearVelocity maxlLinearVelocity) {
    this.linearVelocityLimit = maxlLinearVelocity.in(MetersPerSecond);
  }

  /**
   * Sets the maximum linear acceleration (m/s^2). Limits how fast the robot speeds up, slows down,
   * or changes direction.
   */
  public void setLinearAccelerationLimit(LinearAcceleration maxLinearAcceleration) {
    this.linearAccelerationLimit = maxLinearAcceleration.in(MetersPerSecondPerSecond);
  }

  /** Sets the maximum angular (turning) velocity (rad/s). The robot won't spin faster than this. */
  public void setAngularVelocityLimit(AngularVelocity maxAngularVelocity) {
    this.angularVelocityLimit = maxAngularVelocity.in(RadiansPerSecond);
  }

  /**
   * Sets the maximum angular acceleration (rad/s^2). Limits how fast the robot starts or stops
   * spinning.
   */
  public void setAngularAccelerationLimit(AngularAcceleration maxAngularAcceleration) {
    this.angularAccelerationLimit = maxAngularAcceleration.in(RadiansPerSecondPerSecond);
  }

  /** Removes all drive limits so the robot can use its full speed and acceleration. */
  public void clearDriveLimits() {
    this.linearVelocityLimit = Double.POSITIVE_INFINITY;
    this.linearAccelerationLimit = Double.POSITIVE_INFINITY;
    this.angularVelocityLimit = Double.POSITIVE_INFINITY;
    this.angularAccelerationLimit = Double.POSITIVE_INFINITY;
  }

  /**
   * Applies the configured velocity and acceleration limits to the desired speeds.
   *
   * <p>How it works:
   *
   * <ol>
   *   <li>Acceleration limiting: Compares what the robot wants to do vs. what it did last cycle. If
   *       the change is too large, it scales it down so the robot ramps smoothly.
   *   <li>Velocity limiting: After acceleration is handled, if the speed is still above the cap, we
   *       scale it down to the maximum allowed speed.
   * </ol>
   *
   * <p>This is applied to both linear (driving) and angular (spinning) motion independently.
   */
  private ChassisSpeeds applyDriveLimits(ChassisSpeeds desired) {
    double vx = desired.vxMetersPerSecond;
    double vy = desired.vyMetersPerSecond;
    double omega = desired.omegaRadiansPerSecond;

    // --- Linear acceleration limiting ---
    // Figure out how much the linear velocity is trying to change since last cycle
    double dvx = vx - lastCommandedSpeeds.vxMetersPerSecond;
    double dvy = vy - lastCommandedSpeeds.vyMetersPerSecond;
    double deltaLinearSpeed = Math.hypot(dvx, dvy);

    // The maximum change allowed in one loop cycle = maxAccel * dt
    double maxLinearDelta = linearAccelerationLimit * LOOP_PERIOD_SECS;

    if (deltaLinearSpeed > maxLinearDelta) {
      // Scale the change down so it doesn't exceed the acceleration limit.
      // Think of it like: we want to move toward the target speed, but only take a step
      // of size maxLinearDelta in that direction.
      double scale = maxLinearDelta / deltaLinearSpeed;
      vx = lastCommandedSpeeds.vxMetersPerSecond + dvx * scale;
      vy = lastCommandedSpeeds.vyMetersPerSecond + dvy * scale;
    }

    // --- Angular acceleration limiting ---
    double dOmega = omega - lastCommandedSpeeds.omegaRadiansPerSecond;
    double maxAngularDelta = angularAccelerationLimit * LOOP_PERIOD_SECS;

    if (Math.abs(dOmega) > maxAngularDelta) {
      // Clamp the angular speed change to the max allowed step
      omega = lastCommandedSpeeds.omegaRadiansPerSecond + Math.copySign(maxAngularDelta, dOmega);
    }

    // --- Linear velocity limiting ---
    // Cap the overall driving speed so the robot doesn't exceed the velocity limit
    double linearSpeed = Math.hypot(vx, vy);
    if (linearSpeed > linearVelocityLimit) {
      double scale = linearVelocityLimit / linearSpeed;
      vx *= scale;
      vy *= scale;
    }

    // --- Angular velocity limiting ---
    // Cap the rotation rate so the robot doesn't spin faster than the limit
    omega = MathUtil.clamp(omega, -angularVelocityLimit, angularVelocityLimit);

    ChassisSpeeds limited = new ChassisSpeeds(vx, vy, omega);

    // Remember what we're actually commanding so next cycle can compute acceleration
    lastCommandedSpeeds = limited;

    // Log both the raw request and what we actually sent so we can see the limits in action
    Logger.recordOutput(drivePb.makePath("SwerveChassisSpeeds", "Desired"), desired);
    Logger.recordOutput(drivePb.makePath("SwerveChassisSpeeds", "Limited"), limited);

    return limited;
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "Drive/SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the linear speed of the robot */
  @AutoLogOutput(key = "Drive/MeasuredLinearSpeed")
  public LinearVelocity getSpeed() {
    ChassisSpeeds speeds = getChassisSpeeds();
    double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    return MetersPerSecond.of(linearSpeed);
  }

  @AutoLogOutput(key = "Drive/MeasuredAngularSpeed")
  public AngularVelocity getAngularSpeed() {
    ChassisSpeeds speeds = getChassisSpeeds();
    return RadiansPerSecond.of(speeds.omegaRadiansPerSecond);
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  // Trajectory Following
  public void followTrajectory(SwerveSample sample) {
    // Get the current pose of the robot
    Pose2d pose = getPose();
    Pose2d samplePose2d =
        new Pose2d(
            new Translation2d(sample.x, sample.y), new Rotation2d(Radians.of(sample.heading)));
    Logger.recordOutput("TrajectoryFollowing/pose2d", samplePose2d);
    Logger.recordOutput("TrajectoryFollowing/posex", pose.getX());
    Logger.recordOutput("TrajectoryFollowing/samplex", sample.x);

    Logger.recordOutput("TrajectoryFollowing/posey", pose.getY());
    Logger.recordOutput("TrajectoryFollowing/sampley", sample.y);

    Logger.recordOutput(
        "TrajectoryFollowing/heading", pose.getRotation().getRadians() + Math.PI * 2);
    Logger.recordOutput(
        "TrajectoryFollowing/sampleheading", Rotation2d.fromRadians(sample.heading).getRadians());
    // Generate the next speeds for the robot
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            sample.vx
                + DriveConstants.AutoConstants.xTrajectoryController
                    .createPIDController()
                    .calculate(pose.getX(), sample.x),
            sample.vy
                + DriveConstants.AutoConstants.yTrajectoryController
                    .createPIDController()
                    .calculate(pose.getY(), sample.y),
            sample.omega
                + DriveConstants.AutoConstants.headingTrajectoryController
                    .createAngularPIDController()
                    .calculate(
                        pose.getRotation().getRadians(),
                        Rotation2d.fromRadians(sample.heading).getRadians()));

    this.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, this.getRotation()));
  }
  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }
}
