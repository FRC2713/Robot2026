package frc2713.robot.subsystems.launcher;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2713.lib.io.TalonFXIO;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;

public class Launcher extends SubsystemBase {
  TalonFXIO launcherMotorIO;

  TalonFXSubsystemConfig launcherConfig = new TalonFXSubsystemConfig();

  public Launcher() {
    launcherMotorIO = new TalonFXIO(launcherConfig);
  }

  @Override
  public void periodic() {
    launcherMotorIO.readInputs(null);
  }
}
