package frc2713.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc2713.robot.subsystems.launcher.LauncherConstants.Turret.FORWARD_LIMIT_DEGREES;
import static frc2713.robot.subsystems.launcher.LauncherConstants.Turret.REVERSE_LIMIT_DEGREES;
import static frc2713.robot.subsystems.launcher.LauncherConstants.Turret.SLOPE;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc2713.lib.io.ArticulatedComponent;
import frc2713.lib.subsystem.MotorSubsystem;
import frc2713.lib.util.Util;
import frc2713.robot.FieldConstants;
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
    while (diff <= -180) diff += 360;
    while (diff > 180) diff -= 360;

    // 2. Coarse Estimate (The "Big Picture")
    // This uses the phase shift to guess the rough position.
    double coarseAngle = diff * SLOPE;

    // 3. The Ratio (How many times E1 spins per 1 Turret degree)
    double encoderToTurretRatio = 8.5;

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
    double currentTotalRadians = (current.getRotations() * 2 * Math.PI);
    double closestOffset = Units.degreesToRadians(targetAngleDegrees) - current.getRadians();
    if (closestOffset > Math.PI) {

      closestOffset -= 2 * Math.PI;

    } else if (closestOffset < -Math.PI) {
      closestOffset += 2 * Math.PI;
    }

    double finalOffset = currentTotalRadians + closestOffset;
    if ((currentTotalRadians + closestOffset) % (2 * Math.PI)
        == (currentTotalRadians - closestOffset)
            % (2 * Math.PI)) { // If the offset can go either way, go closer to zero
      if (finalOffset > 0) {
        finalOffset = currentTotalRadians - Math.abs(closestOffset);
      } else {
        finalOffset = currentTotalRadians + Math.abs(closestOffset);
      }
    }
    if (finalOffset
        > Units.degreesToRadians(FORWARD_LIMIT_DEGREES)) { // if past upper rotation limit
      finalOffset -= (2 * Math.PI);
    } else if (finalOffset
        < Units.degreesToRadians(REVERSE_LIMIT_DEGREES)) { // if below lower rotation limit
      finalOffset += (2 * Math.PI);
    }

    return Units.radiansToDegrees(finalOffset);
  }

  /** Input should be robot relative (i.e. encoder-reported angle) */
  public Command setAngle(Supplier<Angle> desiredAngle) {
    return motionMagicSetpointCommand(
        () -> convertSubsystemPositionToMotorPosition(desiredAngle.get()));
  }

  public Command hubCommand(Supplier<Pose2d> robotPose) {
    return setAngle(
        () -> Util.fieldToRobotRelative(LauncherConstants.Turret.staticHubAngle, robotPose.get()));
  }

  /**
   * Gets the current turret position computed from the dual encoder system.
   *
   * @return The computed turret position in degrees
   */
  public Angle getComputedTurretPosition() {
    return inputs.computedTurretPositionDegrees;
  }

  /**
   * Gets the current turret position as a Rotation2d for use with bounded angle calculations.
   *
   * @return The current turret position as a Rotation2d
   */
  public Rotation2d getCurrentTurretRotation() {
    return Rotation2d.fromDegrees(inputs.computedTurretPositionDegrees.in(Degrees));
  }

  /**
   * Supplier that continuously calculates the on-the-fly turret angle. Uses the launch solution if
   * valid, otherwise falls back to simple hub tracking.
   */
  public final Supplier<Angle> otfAngleSupplier =
      () -> {
        Angle currentAngle = Degrees.of(getCurrentTurretRotation().getDegrees());
        var solution = LaunchingSolutionManager.getInstance().getSolution();
        Angle targetAngle =
            solution.isValid() ? Degrees.of(solution.turretFieldYaw().getDegrees()) : currentAngle;

        targetAngle =
            Degrees.of(
                convertToClosestBoundedTurretAngleDegrees(
                    targetAngle.in(Degrees), getCurrentTurretRotation()));
        Logger.recordOutput(super.pb.makePath("OTF", "solutionIsValid"), solution.isValid());
        Logger.recordOutput(pb.makePath("OTF", "targetAngle"), targetAngle);
        return targetAngle;
      };

  public Command otfCommand() {
    return setAngle(otfAngleSupplier);
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
    Logger.recordOutput(pb.makePath("encoder1Degrees"), inputs.encoder1PositionDegrees.in(Degrees));
    Logger.recordOutput(pb.makePath("encoder2Degrees"), inputs.encoder2PositionDegrees.in(Degrees));
    Logger.recordOutput(
        pb.makePath("computedTurretDegrees"), inputs.computedTurretPositionDegrees.in(Degrees));
  }

  @Override
  public Transform3d getTransform3d() {
    Angle rotations = super.getCurrentPosition().times(config.unitToRotorRatio);
    return config.initialTransform.plus(
        new Transform3d(new Translation3d(), new Rotation3d(0, 0, rotations.in(Radians))));
  }

  @Override
  public Translation3d getRelativeAngularVelocity() {
    return new Translation3d(0, 0, super.getCurrentVelocity().in(RadiansPerSecond));
  }
}
