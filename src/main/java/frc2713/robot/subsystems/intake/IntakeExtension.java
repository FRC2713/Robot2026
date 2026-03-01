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
import frc2713.lib.subsystem.MotorSubsystem;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeExtension extends MotorSubsystem<IntakeExtensionInputsAutoLogged, MotorIO>
    implements ArticulatedComponent {

  public IntakeExtension(
      final TalonFXSubsystemConfig config, final MotorIO intakeExtensionMotorIO) {
    super(config, new IntakeExtensionInputsAutoLogged(), intakeExtensionMotorIO);
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

  /**
   * Check if the extension mechanism is at the target position
   *
   * @return true if motion magic has reached the target position
   */
  @AutoLogOutput(key = "Intake Extension/AtTarget")
  public boolean atTarget() {
    return Math.abs(
            getCurrentPositionAsDistance().in(Meters) - getPositionSetpointAsDistance().in(Meters))
        <= IntakeConstants.Extension.acceptableError.in(Meters);
  }

  public Command extendAndWaitCommand() {
    return positionSetpointUntilOnTargetCommand(
        () ->
            convertSubsystemPositionToMotorPosition(
                IntakeConstants.Extension.extendedPosition.get()),
        () -> convertSubsystemPositionToMotorPosition(IntakeConstants.Extension.acceptableError));
  }

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
