package frc2713.lib.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class MotorInputs {
  public AngularVelocity velocityUnitsPerSecond =
      AngularVelocity.ofBaseUnits(0.0, RadiansPerSecond);
  public Angle unitPosition = Angle.ofBaseUnits(0.0, Radians);
  public Voltage appliedVolts = Voltage.ofBaseUnits(0.0, Volts);
  public Current currentStatorAmps = Current.ofBaseUnits(0.0, Amps);
  public Current currentSupplyAmps = Current.ofBaseUnits(0.0, Amps);
  public Angle rawRotorPosition = Angle.ofBaseUnits(0.0, Radians);
}
