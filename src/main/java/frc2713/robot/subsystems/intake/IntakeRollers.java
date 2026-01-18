package frc2713.robot.subsystems.intake;

import frc2713.lib.io.MotorInputsAutoLogged;
import frc2713.lib.io.TalonFXIO;
import frc2713.lib.subsystem.MotorSubsystem;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;

public class IntakeRollers extends MotorSubsystem<MotorInputsAutoLogged, TalonFXIO>  {
    
    public IntakeRollers(
        final TalonFXSubsystemConfig config, final TalonFXIO intakeRollersMotorIO) {
        super(config, new MotorInputsAutoLogged(), intakeRollersMotorIO);
    }



  @Override
  public void periodic() {
    super.periodic();
    // Additional periodic code for intake rollers can be added here
  }
    
}
