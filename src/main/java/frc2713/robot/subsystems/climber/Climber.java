package frc2713.robot.subsystems.climber;

import frc2713.lib.io.MotorInputsAutoLogged;
import frc2713.lib.io.TalonFXIO;
import frc2713.lib.subsystem.MotorSubsystem;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;

public class Climber extends MotorSubsystem<MotorInputsAutoLogged, TalonFXIO> {

  public Climber(final TalonFXSubsystemConfig config, final TalonFXIO climberMotorIO) {
    super(config, new MotorInputsAutoLogged(), climberMotorIO);
  }

  @Override
  public void periodic() {
    super.periodic();
    // Additional periodic code for climber extension can be added here
  }
}
