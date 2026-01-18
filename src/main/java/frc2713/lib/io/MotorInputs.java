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
  public AngularVelocity velocity = AngularVelocity.ofBaseUnits(0.0, RotationsPerSecond);
  public Angle position = Angle.ofBaseUnits(0.0, Rotations);
  public Voltage appliedVolts = Voltage.ofBaseUnits(0.0, Volts);
  public Current currentStatorAmps = Current.ofBaseUnits(0.0, Amps);
  public Current currentSupplyAmps = Current.ofBaseUnits(0.0, Amps);
  public Angle rawRotorPosition = Angle.ofBaseUnits(0.0, Rotations);
}
