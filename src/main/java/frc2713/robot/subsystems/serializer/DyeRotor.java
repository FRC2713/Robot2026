package frc2713.robot.subsystems.serializer;

import frc2713.lib.io.MotorInputsAutoLogged;
import frc2713.lib.io.TalonFXIO;
import frc2713.lib.subsystem.MotorSubsystem;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;

public class DyeRotor extends MotorSubsystem<MotorInputsAutoLogged, TalonFXIO> {

  public DyeRotor(final TalonFXSubsystemConfig config, final TalonFXIO indexerMotorIO) {
    super(config, new MotorInputsAutoLogged(), indexerMotorIO);
  }

  @Override
  public void periodic() {
    super.periodic();
    // Additional periodic code for indexer can be added here
  }
}
