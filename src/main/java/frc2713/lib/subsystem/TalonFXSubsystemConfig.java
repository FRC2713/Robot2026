package frc2713.lib.subsystem;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc2713.lib.drivers.CANDeviceId;

public class TalonFXSubsystemConfig {

  public String name = "UNNAMED";
  public CANDeviceId talonCANID;
  public TalonFXConfiguration fxConfig = new TalonFXConfiguration();
  public DCMotor motor = DCMotor.getKrakenX60(1);
  public ChassisReference simOrientation = ChassisReference.Clockwise_Positive;
  // Marks Slot0 and MotionMagic as live-tunable
  public boolean tunable = false;
  public boolean useFOC = true;

  // Ratio of rotor to units for this talon.  rotor * by this ratio should
  // be the units.
  // <1 is reduction
  public double unitToRotorRatio = 1.0;
  // How far the subsystem travels linearly for one rotation
  public double metersPerRotation = 1.0;
  public double kMinPositionUnits = Double.NEGATIVE_INFINITY;
  public double kMaxPositionUnits = Double.POSITIVE_INFINITY;

  // Moment of Inertia (KgMetersSquared) for sim
  public MomentOfInertia momentOfInertia = KilogramSquareMeters.of(.5);

  public Transform3d initialTransform =
      new Transform3d(new Translation3d(0, 0, 0), new Rotation3d());

  public GeneralControlMode generalControlMode = GeneralControlMode.POSITION;

  public Angle acceptablePositionError = Degree.of(0.5);
  public AngularVelocity acceptableVelocityError = RPM.of(200);

  public static void setLinearAcceptableError(
      TalonFXSubsystemConfig config, double metersPerRotation, Distance acceptableError) {
    config.acceptablePositionError = Rotations.of(acceptableError.in(Meters) / metersPerRotation);
  }

  public enum GeneralControlMode {
    POSITION,
    VELOCITY
  }
}
