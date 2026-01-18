package frc2713.robot.subsystems.launcher;

import frc2713.lib.io.MotorInputsAutoLogged;
import frc2713.lib.io.TalonFXIO;
import frc2713.lib.subsystem.MotorSubsystem;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;

public class Turret extends MotorSubsystem<MotorInputsAutoLogged, TalonFXIO>  {
    
    public Turret(
        final TalonFXSubsystemConfig config, final TalonFXIO turretMotorIO) {
        super(config, new MotorInputsAutoLogged(), turretMotorIO);
    }



  @Override
  public void periodic() {
    super.periodic();
    // Additional periodic code for turret can be added here`
  }
    
}
