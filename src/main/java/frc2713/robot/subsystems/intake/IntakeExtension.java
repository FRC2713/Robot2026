package frc2713.robot.subsystems.intake;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc2713.lib.io.ArticulatedComponent;
import frc2713.lib.subsystem.MotorSubsystem;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;
import frc2713.robot.subsystems.intake.intakeExtensionIO.IntakeExtensionIO;
import frc2713.robot.subsystems.intake.intakeExtensionIO.IntakeExtensionInputsAutoLogged;
import frc2713.robot.subsystems.launcher.LauncherConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class IntakeExtension
    extends MotorSubsystem<IntakeExtensionInputsAutoLogged, IntakeExtensionIO>
    implements ArticulatedComponent {

  public IntakeExtension(
      final TalonFXSubsystemConfig config, final IntakeExtensionIO intakeExtensionMotorIO) {
    super(config, new IntakeExtensionInputsAutoLogged(), intakeExtensionMotorIO);
  }

  /*
   * Move to specified distance with motion magic and custom cruise velocity
   */
  public Command setDistanceCommand(
      Supplier<Distance> desiredDistance, Supplier<LinearVelocity> cruiseVelocity) {
    return Commands.sequence(
        new InstantCommand(
            () -> {
              AngularVelocity cruiseAngularVelocity =
                  convertSubsystemVelocityToMotorVelocity(cruiseVelocity.get());
              Logger.recordOutput(pb.makePath("cruiseLinearVelocity"), cruiseVelocity.get());
              Logger.recordOutput(pb.makePath("cruiseAngularVelocity"), cruiseAngularVelocity);
              setMotionMagicConfigImpl(
                  IntakeConstants.Extension.motionMagicGains
                      .get()
                      .withMotionMagicCruiseVelocity(cruiseAngularVelocity));
            }),
        motionMagicSetpointCommand(
            () -> convertSubsystemPositionToMotorPosition(desiredDistance.get())));
  }

  /**
   * Move to specified distance with motion magic
   *
   * @param desiredDistance
   * @return
   */
  public Command setDistanceCommand(Supplier<Distance> desiredDistance) {
    return setDistanceCommand(
        desiredDistance,
        () ->
            convertMotorVelocityToSubsystemVelocity(
                IntakeConstants.Extension.motionMagicGains
                    .get()
                    .getMotionMagicCruiseVelocityMeasure()));
  }

  /**
   * Move to the extended position with motion magic, does not stop until setpoint is reached
   *
   * @return
   */
  public Command extendFullyCommand() {
    return Commands.repeatingSequence(
            setDistanceCommand(IntakeConstants.Extension.extendedPosition))
        .until(() -> this.atTarget());
  }

  /**
   * Move to the extended position with motion magic, does not stop until setpoint is reached
   *
   * @return
   */
  public Command retractFullyCommand() {
    return Commands.repeatingSequence(
            setDistanceCommand(IntakeConstants.Extension.retractedPosition))
        .until(() -> this.atTarget());
  }

  public Command extendCommand() {
    return setDistanceCommand(IntakeConstants.Extension.extendedPosition);
  }

  public Command retractCommand() {
    return setDistanceCommand(
        IntakeConstants.Extension.retractedPosition,
        IntakeConstants.Extension.retractCruiseVelocity);
  }

  /**
   * Maintain fuel pressure by retracting the intake at a rate that shrinks the hopper volume by the
   * same rate that fuel is leaving the hopper
   *
   * @return A Command that maintains fuel pressure by retracting the intake at a rate that shrinks
   *     the hopper volume by the same rate that fuel is leaving the hopper
   * @see LauncherConstants.launchRateVolumeInchesCubedPerSecond
   */
  public Command maintainFuelPressureCommand() {
    double volumeLostPerSecond = LauncherConstants.Flywheels.launchRateVolumeInchesCubedPerSecond;
    Logger.recordOutput(pb.makePath("volumeLostPerSecond (in^3)"), volumeLostPerSecond);
    Logger.recordOutput(
        pb.makePath("volumePerInch (in^3)"), IntakeConstants.Extension.volumePerInch);
    LinearVelocity velocityToMaintain =
        InchesPerSecond.of(volumeLostPerSecond / IntakeConstants.Extension.volumePerInch / 2.0);
    return setDistanceCommand(IntakeConstants.Extension.retractedPosition, () -> velocityToMaintain)
        .withName("Maintain Fuel Pressure");
  }

  public Command extendAndWaitCommand() {
    return positionSetpointUntilOnTargetCommand(
        () ->
            convertSubsystemPositionToMotorPosition(
                IntakeConstants.Extension.extendedPosition.get()));
  }

  @Override
  public void periodic() {
    io.readInputs(inputs);
    super.periodic();

    Logger.recordOutput(pb.makePath("CurrentDistanceMeters"), getCurrentPositionAsDistance());
    Logger.recordOutput(pb.makePath("SetpointDistanceMeters"), getPositionSetpointAsDistance());

    Logger.recordOutput(pb.makePath("AtTarget"), atTarget());

    Logger.recordOutput(
        pb.makePath("LinearVelocity"),
        convertMotorVelocityToSubsystemVelocity(getCurrentVelocity()));

    if (DriverStation.isDisabled()) {
      setPositionSetpointImpl(inputs.position);
    }
  }

  @Override
  public Transform3d getTransform3d() {
    Distance distance = getCurrentPositionAsDistance();

    return new Transform3d(new Translation3d(distance.in(Meters), 0, 0), new Rotation3d());
  }
}
