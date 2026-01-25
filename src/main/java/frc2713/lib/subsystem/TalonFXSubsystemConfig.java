package frc2713.lib.subsystem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc2713.lib.drivers.CANDeviceId;

public class TalonFXSubsystemConfig {

  public String name = "UNNAMED";
  public CANDeviceId talonCANID;
  public TalonFXConfiguration fxConfig = new TalonFXConfiguration();

  // Ratio of rotor to units for this talon.  rotor * by this ratio should
  // be the units.
  // <1 is reduction
  public double unitToRotorRatio = 1.0;
  // The number of times the pulley, drum, etc has to rotate to achieve
  // 1m of travel
  public double unitRotationsPerMeter = 1.0;
  public double kMinPositionUnits = Double.NEGATIVE_INFINITY;
  public double kMaxPositionUnits = Double.POSITIVE_INFINITY;

  // Moment of Inertia (KgMetersSquared) for sim
  public double momentOfInertia = 0.5;
}
