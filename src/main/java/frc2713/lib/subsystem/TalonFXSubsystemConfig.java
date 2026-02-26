package frc2713.lib.subsystem;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc2713.lib.drivers.CANDeviceId;

public class TalonFXSubsystemConfig {

  public String name = "UNNAMED";
  public CANDeviceId talonCANID;
  public TalonFXConfiguration fxConfig = new TalonFXConfiguration();
  // Marks Slot0 and MotionMagic as live-tunable
  public boolean tunable = false;
  public boolean useFOC = true;

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
  public MomentOfInertia momentOfInertia = KilogramSquareMeters.of(.5);

  public Transform3d initialTransform =
      new Transform3d(new Translation3d(0, 0, 0), new Rotation3d());
}
