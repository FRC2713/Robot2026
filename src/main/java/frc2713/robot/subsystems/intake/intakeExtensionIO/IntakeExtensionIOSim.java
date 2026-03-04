package frc2713.robot.subsystems.intake.intakeExtensionIO;

import frc2713.lib.io.SimTalonFXIO;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;

public class IntakeExtensionIOSim extends SimTalonFXIO implements IntakeExtensionIO {
  public IntakeExtensionIOSim(TalonFXSubsystemConfig config) {
    super(config);
  }
}
