package frc2713.robot.subsystems.launcher.turretIO;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import frc2713.lib.io.MotorInputsAutoLogged;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class TurretInputs extends MotorInputsAutoLogged {
  /** Position from encoder 1 (TalonFX integrated encoder) in degrees */
  public Angle encoder1Position = Degrees.of(0.0);

  /** Position from encoder 2 (external CANCoder) in degrees */
  public Angle encoder2Position = Degrees.of(0.0);

  /** Computed turret absolute position from dual encoder calculation */
  public Angle computedTurretPosition = Degrees.of(0.0);
}
