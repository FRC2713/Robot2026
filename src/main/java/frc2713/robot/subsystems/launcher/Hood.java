package frc2713.robot.subsystems.launcher;

import frc2713.lib.io.MotorInputsAutoLogged;
import frc2713.lib.io.TalonFXIO;
import frc2713.lib.subsystem.MotorSubsystem;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;

public class Hood extends MotorSubsystem<MotorInputsAutoLogged, TalonFXIO> {

  public Hood(final TalonFXSubsystemConfig config, final TalonFXIO launcherMotorIO) {
    super(config, new MotorInputsAutoLogged(), launcherMotorIO);
  }

  @Override
  public void periodic() {
    super.periodic();
    // Additional periodic code for Hood can be added here
  }
}
