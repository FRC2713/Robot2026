package frc2713.robot.subsystems.intake;

import frc2713.lib.io.MotorInputsAutoLogged;
import frc2713.lib.io.TalonFXIO;
import frc2713.lib.subsystem.MotorSubsystem;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;

public class IntakeExtension extends MotorSubsystem<MotorInputsAutoLogged, TalonFXIO> {

  public IntakeExtension(
      final TalonFXSubsystemConfig config, final TalonFXIO intakeExtensionMotorIO) {
    super(config, new MotorInputsAutoLogged(), intakeExtensionMotorIO);
  }

  @Override
  public void periodic() {
    super.periodic();
    // Additional periodic code for intake can be added here
  }
}
