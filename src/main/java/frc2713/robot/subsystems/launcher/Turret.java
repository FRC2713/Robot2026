package frc2713.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc2713.robot.subsystems.launcher.LauncherConstants.Turret.ENCODER_1_TO_TURRET_RATIO;
import static frc2713.robot.subsystems.launcher.LauncherConstants.Turret.FORWARD_LIMIT_DEGREES;
import static frc2713.robot.subsystems.launcher.LauncherConstants.Turret.REVERSE_LIMIT_DEGREES;
import static frc2713.robot.subsystems.launcher.LauncherConstants.Turret.SLOPE;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc2713.lib.io.ArticulatedComponent;
import frc2713.lib.subsystem.MotorSubsystem;
import frc2713.lib.util.Util;
import frc2713.robot.FieldConstants;
import frc2713.robot.RobotContainer;
import frc2713.robot.subsystems.launcher.turretIO.TurretInputsAutoLogged;
import frc2713.robot.subsystems.launcher.turretIO.TurretMotorIO;
import frc2713.robot.subsystems.launcher.turretIO.TurretSubsystemConfig;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends MotorSubsystem<TurretInputsAutoLogged, TurretMotorIO>
    implements ArticulatedComponent {

  public Turret(final TurretSubsystemConfig config, final TurretMotorIO turretMotorIO) {
    super(config, new TurretInputsAutoLogged(), turretMotorIO);
  }

  public static double turretPositionFromEncoders(double e1, double e2) {
    // 1. Calculate the 'Difference' (The Phase)
    double diff = e2 - e1;

    // Normalize to [-180, 180]. This is the "Vernier Lock"
    diff = MathUtil.inputModulus(diff, -180, 180);

    // 2. Coarse Estimate (The "Big Picture")
    // This uses the phase shift to guess the rough position.
    double coarseAngle = diff * SLOPE;

    // 3. The Ratio (How many times E1 spins per 1 Turret degree)
    double encoderToTurretRatio = ENCODER_1_TO_TURRET_RATIO;

    // 4. Lap Calculation (The "Fine" logic)
    // We calculate how many full 360s E1 has likely traveled.
    // 'expectedE1' is where the encoder SHOULD be if coarseAngle was perfect.
    double expectedE1 = coarseAngle * encoderToTurretRatio;

    // Determine the 'Lap' by finding how many 360s are between
    // the expected position and the actual sensor reading (e1).
    double lap = Math.round((expectedE1 - e1) / 360.0);

    // 5. High-Resolution Output
    // Combine the Lap (coarse) with the Sensor Reading (fine)
    return (lap * 360.0 + e1) / encoderToTurretRatio;
  }

  public static double convertToClosestBoundedTurretAngleDegrees(
      double targetAngleDegrees, Rotation2d current) {
    // Normalize target to [-180, 180] first
    double normalizedTarget = MathUtil.inputModulus(targetAngleDegrees, -180, 180);

    // Get current position in degrees
    double currentDegrees = current.getDegrees();

    // Calculate the shortest path to the target (normalized to [-180, 180])
    double diff = MathUtil.inputModulus(normalizedTarget - currentDegrees, -180, 180);

    // Calculate the final absolute position
    double finalPosition = currentDegrees + diff;

    // Check if final position is within limits, if not, try the other way around
    if (finalPosition > FORWARD_LIMIT_DEGREES) {
      finalPosition -= 360;
    } else if (finalPosition < REVERSE_LIMIT_DEGREES) {
      finalPosition += 360;
    }

    return finalPosition;
  }

  /** Input should be robot relative (i.e. encoder-reported angle) */
  public Command setAngle(Supplier<Angle> desiredAngle) {
    return motionMagicSetpointCommand(
        () -> {
          double commandedDegrees = desiredAngle.get().in(Degrees);

          // Convert the desired angle to a bounded angle that respects turret limits
          double boundedAngleDegrees =
              convertToClosestBoundedTurretAngleDegrees(
                  commandedDegrees, getCurrentTurretRotation());

          Logger.recordOutput(pb.makePath("setpoint", "commandedDegrees"), commandedDegrees);
          Logger.recordOutput(pb.makePath("setpoint", "boundedDegrees"), boundedAngleDegrees);

          return convertSubsystemPositionToMotorPosition(Degrees.of(boundedAngleDegrees));
        });
  }

  /**
   * Gets the current turret position computed from the dual encoder system.
   *
   * @return The computed turret position in degrees
   */
  public Angle getComputedTurretPosition() {
    return inputs.computedTurretPosition;
  }

  /**
   * Gets the current turret position as a Rotation2d for use with bounded angle calculations.
   *
   * @return The current turret position as a Rotation2d
   */
  public Rotation2d getCurrentTurretRotation() {
    return Rotation2d.fromDegrees(inputs.computedTurretPosition.in(Degrees));
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
          targetAngle = Degrees.of(solution.turretFieldYaw().getDegrees());
          Logger.recordOutput(super.pb.makePath("OTF", "response"), "using solution");
        } else if (solution.effectiveDistanceMeters() <= 0.9) {
          // invalid bc we're too close
          Logger.recordOutput(super.pb.makePath("OTF", "response"), "hub shot");
          targetAngle =
              Util.fieldToRobotRelative(
                  LauncherConstants.Turret.staticHubAngle, RobotContainer.drive.getPose());
        } else {
          Logger.recordOutput(super.pb.makePath("OTF", "response"), "stay at measured");
          targetAngle = inputs.computedTurretPosition;
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

    // Read turret-specific inputs (includes dual encoder data)
    io.readInputs(inputs);

    // Log the goal pose for visualization
    Pose3d goalPose = new Pose3d(FieldConstants.Hub.topCenterPoint, new Rotation3d());
    Logger.recordOutput(pb.makePath("goalVector"), new Pose3d[] {this.getGlobalPose(), goalPose});
  }

  @Override
  public Transform3d getTransform3d() {
    // Use the computed turret position from the Vernier dual-encoder system (in degrees)
    double turretAngleRadians = inputs.computedTurretPosition.in(Radian);
    return config.initialTransform.plus(
        new Transform3d(new Translation3d(), new Rotation3d(0, 0, turretAngleRadians)));
  }

  @Override
  public Translation3d getRelativeAngularVelocity() {
    return new Translation3d(0, 0, super.getCurrentVelocity().in(RadiansPerSecond));
  }
}
