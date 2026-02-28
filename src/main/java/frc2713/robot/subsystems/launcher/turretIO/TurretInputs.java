package frc2713.robot.subsystems.launcher.turretIO;

import frc2713.lib.io.MotorInputs;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class TurretInputs extends MotorInputs {
  /** Position from encoder 1 (TalonFX integrated encoder) in degrees */
  public double encoder1PositionDegrees = 0.0;

  /** Position from encoder 2 (external CANCoder) in degrees */
  public double encoder2PositionDegrees = 0.0;

  /** Computed turret absolute position from dual encoder calculation in degrees */
  public double computedTurretPositionDegrees = 0.0;
}
