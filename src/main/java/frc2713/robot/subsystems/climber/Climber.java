package frc2713.robot.subsystems.climber;

import frc2713.lib.io.MotorIO;
import frc2713.lib.io.MotorInputsAutoLogged;
import frc2713.lib.subsystem.MotorSubsystem;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;

public class Climber extends MotorSubsystem<MotorInputsAutoLogged, MotorIO> {

  public Climber(final TalonFXSubsystemConfig config, final MotorIO climberMotorIO) {
    super(config, new MotorInputsAutoLogged(), climberMotorIO);
  }

  @Override
  public void periodic() {
    super.periodic();
    // Additional periodic code for climber extension can be added here
  }
}
