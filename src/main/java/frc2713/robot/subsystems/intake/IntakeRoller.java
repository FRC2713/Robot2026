package frc2713.robot.subsystems.intake;

import frc2713.lib.io.MotorIO;
import frc2713.lib.io.MotorInputsAutoLogged;
import frc2713.lib.subsystem.MotorSubsystem;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;

public class IntakeRoller extends MotorSubsystem<MotorInputsAutoLogged, MotorIO> {

  public IntakeRoller(final TalonFXSubsystemConfig config, final MotorIO intakeRollersMotorIO) {
    super(config, new MotorInputsAutoLogged(), intakeRollersMotorIO);
  }

  @Override
  public void periodic() {
    super.periodic();
    // Additional periodic code for intake rollers can be added here
  }
}
