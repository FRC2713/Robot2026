package frc2713.lib.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class MotorInputs {
  public AngularVelocity velocity = RotationsPerSecond.of(0.0);
  public Angle position = Rotations.of(0.0);
  public Voltage appliedVolts = Volts.of(0.0)
  public Current currentStatorAmps = Amps.of(0.0);
  public Current currentSupplyAmps = Amps.of(0.0);
  public Angle rawRotorPosition = Rotations.of(0.0);
}
