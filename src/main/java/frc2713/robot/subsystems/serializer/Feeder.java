package frc2713.robot.subsystems.serializer;

import frc2713.lib.io.MotorIO;
import frc2713.lib.io.MotorInputsAutoLogged;
import frc2713.lib.subsystem.MotorSubsystem;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;

public class Feeder extends MotorSubsystem<MotorInputsAutoLogged, MotorIO> {

  public Feeder(final TalonFXSubsystemConfig config, final MotorIO feederMotorIO) {
    super(config, new MotorInputsAutoLogged(), feederMotorIO);
  }

  @Override
  public void periodic() {
    super.periodic();
    // Additional periodic code for feeder can be added here
  }
}
