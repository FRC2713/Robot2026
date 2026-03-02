package frc2713.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc2713.lib.geometry.GeometryUtil;
import frc2713.lib.io.ArticulatedComponent;
import frc2713.lib.io.CanCoderIO;
import frc2713.lib.io.CanCoderInputsAutoLogged;
import frc2713.lib.io.MotorIO;
import frc2713.lib.io.MotorInputsAutoLogged;
import frc2713.lib.subsystem.MotorSubsystem;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;
import frc2713.lib.util.CrtSolver;
import frc2713.lib.util.Util;
import frc2713.robot.FieldConstants;
import frc2713.robot.RobotContainer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends MotorSubsystem<MotorInputsAutoLogged, MotorIO>
    implements ArticulatedComponent {

  /**
   * Constructor for Turret with CANcoder. Pass null for cancoderInputs and cancoderIO when using
   * sim or replay (TurretMotorIOSim computes encoder2 from encoder1).
   */
  public Turret(
      final TalonFXSubsystemConfig config,
      final MotorIO turretMotorIO,
      final CanCoderInputsAutoLogged cancoderInputs,
      final CanCoderIO cancoderIO) {
    super(config, new MotorInputsAutoLogged(), turretMotorIO, cancoderInputs, cancoderIO);
  }

  public static Angle turretPositionFromEncoders(Angle e1, Angle e2) {
    Angle encoder1Rotations =
        CrtSolver.calculateAbsoluteMotorTurns(
            e1,
            e2,
            LauncherConstants.Turret.pinionGearTeeth,
            LauncherConstants.Turret.spurGear1Teeth);
    return encoder1Rotations.div(LauncherConstants.Turret.motorToTurretGearRatio);
  }

  public static Angle convertToClosestBoundedTurretAngleDegrees(Angle desiredAngle, Angle current) {
    // Normalize target to [-180, 180] first
    Angle normalizedTarget =
        GeometryUtil.angleModulus(desiredAngle, Degrees.of(-180), Degrees.of(180));

    // Calculate the shortest path to the target (normalized to [-180, 180])

    Angle diff =
        GeometryUtil.angleModulus(
            normalizedTarget.minus(current), Degrees.of(-180), Degrees.of(180));

    // Calculate the final absolute position
    Angle finalPosition = current.plus(diff);

    // Check if final position is within limits, if not, try the other way around
    if (finalPosition.gt(LauncherConstants.Turret.forwardSoftLimit)) {
      finalPosition = finalPosition.minus(Rotations.of(1));
    } else if (finalPosition.lt(LauncherConstants.Turret.reverseSoftLimit)) {
      finalPosition = finalPosition.plus(Rotations.of(1));
    }

    return finalPosition;
  }

  /** Input should be robot relative (i.e. encoder-reported angle) */
  public Command setAngle(Supplier<Angle> desiredAngle) {
    return motionMagicSetpointCommand(
        () -> {

          // Convert the desired angle to a bounded angle that respects turret limits
          Angle boundedAngleDegrees =
              convertToClosestBoundedTurretAngleDegrees(
                  desiredAngle.get(), getCurrentTurretRotation());

          Logger.recordOutput(
              pb.makePath("setpoint", "commandedAngle"), desiredAngle.get().in(Degrees));
          Logger.recordOutput(pb.makePath("setpoint", "boundedAngle"), boundedAngleDegrees);

          return Degrees.of(boundedAngleDegrees);
        });
  }

  /**
   * Like {@link #setAngle}, but clamps to the turret limits instead of wrapping around. Useful for
   * manual rotation where you want the turret to stop at the bounds.
   */
  public Command setAngleStopAtBounds(Supplier<Angle> desiredAngle) {
    return motionMagicSetpointCommand(
        () -> {
          double commandedDegrees = desiredAngle.get().in(Degrees);

          // Clamp directly to turret limits instead of wrapping
          double clampedDegrees =
              MathUtil.clamp(commandedDegrees, REVERSE_LIMIT_DEGREES, FORWARD_LIMIT_DEGREES);

          Logger.recordOutput(pb.makePath("setpoint", "commandedDegrees"), commandedDegrees);
          Logger.recordOutput(pb.makePath("setpoint", "clampedDegrees"), clampedDegrees);

          return Degrees.of(clampedDegrees);
        });
  }

  /**
   * Like {@link #setAngleStopAtBounds}, but allows scaling the velocity and acceleration based on
   * an input (e.g., trigger pressure).
   *
   * @param desiredAngle The desired angle supplier
   * @param velocityScale Scale factor for velocity and acceleration (0.0 to 1.0)
   */
  public Command setAngleStopAtBounds(
      Supplier<Angle> desiredAngle, Supplier<Double> velocityScale) {
    return motionMagicSetpointCommand(
        () -> {
          double commandedDegrees = desiredAngle.get().in(Degrees);

          // Clamp directly to turret limits instead of wrapping
          double clampedDegrees =
              MathUtil.clamp(commandedDegrees, REVERSE_LIMIT_DEGREES, FORWARD_LIMIT_DEGREES);

          Logger.recordOutput(pb.makePath("setpoint", "commandedDegrees"), commandedDegrees);
          Logger.recordOutput(pb.makePath("setpoint", "clampedDegrees"), clampedDegrees);

          return Degrees.of(clampedDegrees);
        },
        () -> {
          var mmConfig = new com.ctre.phoenix6.configs.MotionMagicConfigs();
          double scale = MathUtil.clamp(velocityScale.get(), 0.0, 1.0);

          // Scale velocity and acceleration based on input
          mmConfig.MotionMagicCruiseVelocity =
              config.fxConfig.MotionMagic.MotionMagicCruiseVelocity * scale * .5;
          mmConfig.MotionMagicAcceleration =
              config.fxConfig.MotionMagic.MotionMagicAcceleration * scale * .5;
          mmConfig.MotionMagicJerk = config.fxConfig.MotionMagic.MotionMagicJerk;

          Logger.recordOutput(pb.makePath("setpoint", "velocityScale"), scale);

          return mmConfig;
        },
        0);
  }

  /**
   * Gets the current turret position computed from the dual encoder system.
   *
   * @return The computed turret position in degrees
   */
  public Angle getComputedTurretPosition() {
    if (cancoderInputs == null) {
      return Degrees.of(inputs.position.in(Degrees));
    }
    return Degrees.of(
        Double.isNaN(cancoderInputs.absolutePositionRotations)
            ? 0.0
            : cancoderInputs.absolutePositionRotations * 360.0);
  }

  /**
   * Gets the current turret position as a Rotation2d for use with bounded angle calculations.
   *
   * @return The current turret position as a Rotation2d
   */
  public Angle getCurrentTurretRotation() {
    if (cancoderInputs == null) {
      return Degrees.of(inputs.position.in(Degrees));
    }
    return Degrees.of(
        Double.isNaN(cancoderInputs.absolutePositionRotations)
            ? 0.0
            : cancoderInputs.absolutePositionRotations);
  }

  /**
   * Supplier that continuously calculates the on-the-fly turret angle. Uses the launch solution if
   * valid, otherwise falls back to aiming at the goal using simple geometry.
   */
  public final Supplier<Angle> otfAngleSupplier =
      () -> {
        var solution = LaunchingSolutionManager.getInstance().getSolution();
        Angle targetAngle;

        if (solution.isValid()) {
          // Convert the field-relative yaw to robot-relative using the current robot heading.
          // This must happen here (not in LaunchingSolutionManager) so we always use the
          // robot's heading RIGHT NOW, not a projected or stale heading from a previous cycle.
          targetAngle =
              Util.fieldToRobotRelative(
                  Degrees.of(solution.turretFieldYaw().getDegrees()),
                  RobotContainer.drive.getPose());
          Logger.recordOutput(super.pb.makePath("OTF", "response"), "using solution");
        } else if (solution.effectiveDistanceMeters() <= 0.9) {
          // invalid bc we're too close
          Logger.recordOutput(super.pb.makePath("OTF", "response"), "hub shot");
          targetAngle =
              Util.fieldToRobotRelative(
                  LauncherConstants.Turret.staticHubAngle, RobotContainer.drive.getPose());
        } else {
          Logger.recordOutput(super.pb.makePath("OTF", "response"), "stay at measured");
          targetAngle = getComputedTurretPosition();
        }

        targetAngle =
            Degrees.of(
                convertToClosestBoundedTurretAngleDegrees(
                    targetAngle.in(Degrees), getCurrentTurretRotation()));
        Logger.recordOutput(super.pb.makePath("OTF", "solutionIsValid"), solution.isValid());
        Logger.recordOutput(pb.makePath("OTF", "targetAngleDegrees"), targetAngle.in(Degrees));
        return targetAngle;
      };

  public final Supplier<Angle> hubAngleSupplier =
      () -> {
        Angle val = LauncherConstants.Turret.staticHubAngle;
        if (DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red) {
          val = val.plus(Degrees.of(180.0));
        }
        return val;
      };

  public Command otfCommand() {
    return setAngle(otfAngleSupplier);
  }

  public Command hubCommand(Supplier<Pose2d> robotPose) {
    return setAngle(() -> Util.fieldToRobotRelative(hubAngleSupplier.get(), robotPose.get()));
  }

  @AutoLogOutput
  public boolean atTarget() {
    return this.inputs.isMotionMagicAtTarget;
  }

  @Override
  public void periodic() {
    super.periodic();
    this.io.readInputs(inputs);

    // Log the goal pose for visualization
    Pose3d goalPose = new Pose3d(FieldConstants.Hub.topCenterPoint, new Rotation3d());
    Logger.recordOutput(pb.makePath("goalVector"), new Pose3d[] {this.getGlobalPose(), goalPose});
  }

  @Override
  public Transform3d getTransform3d() {
    // Use the computed turret position from the Vernier dual-encoder system (in degrees)
    double turretAngleRadians = getCurrentTurretRotation().getRadians();
    return config.initialTransform.plus(
        new Transform3d(new Translation3d(), new Rotation3d(0, 0, turretAngleRadians)));
  }

  @Override
  public Translation3d getRelativeAngularVelocity() {
    return new Translation3d(0, 0, super.getCurrentVelocity().in(RadiansPerSecond));
  }
}
