package frc2713.robot.subsystems.intake;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc2713.lib.io.MotorInputsAutoLogged;
import frc2713.lib.io.TalonFXIO;
import frc2713.lib.subsystem.MotorSubsystem;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;

public class IntakeExtension extends MotorSubsystem<MotorInputsAutoLogged, TalonFXIO> {

  public IntakeExtension(
      final TalonFXSubsystemConfig config, final TalonFXIO intakeExtensionMotorIO) {
    super(config, new MotorInputsAutoLogged(), intakeExtensionMotorIO);
  }

  /**
   * Move to specified distance with motion magic
   * @param desiredDistance
   * @return
   */
  public Command setDistanceCommand(Supplier<Distance> desiredDistance) {
    return motionMagicSetpointCommand(() -> convertSubsystemPositionToMotorPosition(desiredDistance.get()));
  }

  /**
   * Move to the extended position with motion magic
   * @return
   */
  public Command extendCommand() {
    return setDistanceCommand(() -> IntakeConstants.Extension.extendedPostion);
  }

  /**
   * Move to the retracted postion with motion magic
   * @return
   */
  public Command retractCommand() {
    return setDistanceCommand(() -> IntakeConstants.Extension.retractedPosition);
  }

  @Override
  public void periodic() {
    super.periodic();
    // Additional periodic code for intake can be added here
  }
}
