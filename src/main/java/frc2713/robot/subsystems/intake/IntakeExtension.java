package frc2713.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc2713.lib.io.ArticulatedComponent;
import frc2713.lib.io.MotorIO;
import frc2713.lib.io.MotorInputsAutoLogged;
import frc2713.lib.subsystem.MotorSubsystem;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class IntakeExtension extends MotorSubsystem<MotorInputsAutoLogged, MotorIO>
    implements ArticulatedComponent {

  public IntakeExtension(
      final TalonFXSubsystemConfig config, final MotorIO intakeExtensionMotorIO) {
    super(config, new MotorInputsAutoLogged(), intakeExtensionMotorIO);
  }

  /**
   * Move to specified distance with motion magic
   *
   * @param desiredDistance
   * @return
   */
  public Command setDistanceCommand(Supplier<Distance> desiredDistance) {
    return motionMagicSetpointCommand(
        () -> convertSubsystemPositionToMotorPosition(desiredDistance.get()));
  }

  /**
   * Move to the extended position with motion magic
   *
   * @return
   */
  public Command extendCommand() {
    return setDistanceCommand(IntakeConstants.Extension.extendedPosition);
  }

  /**
   * Move to the retracted postion with motion magic
   *
   * @return
   */
  public Command retractCommand() {
    return setDistanceCommand(IntakeConstants.Extension.retractedPosition);
  }

  // TODO: add is at target

  @Override
  public void periodic() {
    super.periodic();

    Logger.recordOutput(
        pb.makePath("CurrentDistanceMeters"), getCurrentPositionAsDistance().in(Meters));
    Logger.recordOutput(
        pb.makePath("SetpointDistanceMeters"), getPositionSetpointAsDistance().in(Meters));

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
