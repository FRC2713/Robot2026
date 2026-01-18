package frc2713.robot.subsystems.indexer;

import frc2713.lib.io.MotorInputsAutoLogged;
import frc2713.lib.io.TalonFXIO;
import frc2713.lib.subsystem.MotorSubsystem;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;

public class Indexer extends MotorSubsystem<MotorInputsAutoLogged, TalonFXIO>  {
    
    public Indexer(
        final TalonFXSubsystemConfig config, final TalonFXIO indexerMotorIO) {
        super(config, new MotorInputsAutoLogged(), indexerMotorIO);
    }



  @Override
  public void periodic() {
    super.periodic();
    // Additional periodic code for indexer can be added here
  }
    
}
