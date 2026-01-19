package frc2713.robot.subsystems.serializer;

import frc2713.lib.io.MotorInputsAutoLogged;
import frc2713.lib.io.TalonFXIO;
import frc2713.lib.subsystem.MotorSubsystem;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;

public class Feeder extends MotorSubsystem<MotorInputsAutoLogged, TalonFXIO> {

  public Feeder(final TalonFXSubsystemConfig config, final TalonFXIO feederMotorIO) {
    super(config, new MotorInputsAutoLogged(), feederMotorIO);
  }

  @Override
  public void periodic() {
    super.periodic();
    // Additional periodic code for feeder can be added here
  }
}
