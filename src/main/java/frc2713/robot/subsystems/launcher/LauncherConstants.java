package frc2713.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import frc2713.lib.drivers.CANDeviceId;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;

public final class LauncherConstants {

  public static final class Turret {

    public static TurretSubsystemConfig config = new TurretSubsystemConfig();

    static {
      config.name = "Turret";
      config.talonCANID = new CANDeviceId(12); // Example CAN ID, replace with actual ID
      config.canCoderCANID = new CANDeviceId(13); // CANCoder CAN ID, replace with actual ID
      config.fxConfig.Slot0.kP = 0.2;
      config.fxConfig.Slot0.kI = 0.0;
      config.fxConfig.Slot0.kD = 0.0;
      // rotations of subsystem for every 1 rotation of motor
      config.unitToRotorRatio = 1.0; // 1:1 ratio
      config.initialTransform =
          new Transform3d(
              new Translation3d(0, Inches.of(1.75).in(Meters), Inches.of(18.484119).in(Meters)),
              new Rotation3d(0, 0, 0));
    }

    public static int MODEL_INDEX = 3;
    public static int PARENT_INDEX = 0; // drivetrain

    // Gear tooth counts for turret angle calculation
    // Pinion on motor
    public static final double GEAR_0_TOOTH_COUNT = 60.0; // TODO: Replace with actual value
    // attached to e1
    public static final double GEAR_1_TOOTH_COUNT = 120.0; // TODO: Replace with actual value
    // attached to e2
    public static final double GEAR_2_TOOTH_COUNT = 80.0; // TODO: Replace with actual value
    public static final double SLOPE =
        (GEAR_2_TOOTH_COUNT * GEAR_1_TOOTH_COUNT)
            / ((GEAR_1_TOOTH_COUNT - GEAR_2_TOOTH_COUNT) * GEAR_0_TOOTH_COUNT);

    // Turret rotation limits
    public static final double FORWARD_LIMIT_DEGREES = 180.0; // TODO: Replace with actual value
    public static final double REVERSE_LIMIT_DEGREES = -180.0; // TODO: Replace with actual value
  }

  public final class Flywheels {

    public static TalonFXSubsystemConfig config = new TalonFXSubsystemConfig();

    static {
      config.name = "Flywheels";
      config.talonCANID = new CANDeviceId(13); // Example CAN ID, replace with actual ID
      config.fxConfig.Slot0.kP = 0.2;
      config.fxConfig.Slot0.kI = 0.0;
      config.fxConfig.Slot0.kD = 0.0;
      config.unitToRotorRatio = 1.0; // 1:1 ratio
    }

    public static int MODEL_INDEX = 5;
    public static int PARENT_INDEX = 4;

    public static Transform3d localTransform =
        new Transform3d(
            new Translation3d(Inches.of(-5).in(Meters), 0, Inches.of(2).in(Meters)),
            new Rotation3d(0, Degrees.of(-90).in(Radians), 0));

    public static InterpolatingDoubleTreeMap velocityMap = new InterpolatingDoubleTreeMap();

    static {
      // Distance (m) -> Ball Velocity (ft/s)
      velocityMap.put(1.0, 20.0);
      velocityMap.put(1.5, 20.0);
      velocityMap.put(3.0, 23.0);
      velocityMap.put(4.0, 25.0);
      velocityMap.put(5.17, 28.0);
    }
  }

  public final class Hood {

    public static TalonFXSubsystemConfig config = new TalonFXSubsystemConfig();

    static {
      config.name = "Hood";
      config.talonCANID = new CANDeviceId(14); // Example CAN ID, replace with actual ID
      config.fxConfig.Slot0.kP = 0.2;
      config.fxConfig.Slot0.kI = 0.0;
      config.fxConfig.Slot0.kD = 0.0;
      config.unitToRotorRatio = 1.0; // 1:1 ratio
      config.initialTransform =
          new Transform3d(
              new Translation3d(Inches.of(4.086915).in(Meters), 0, 0), new Rotation3d());
    }

    public static Angle retractedPosition = Degrees.of(0);
    public static int MODEL_INDEX = 4;
    public static int PARENT_INDEX = 3; // turret

    public static InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();

    static {
      // Distance (m) -> Hood Pitch (Degrees)
      angleMap.put(1.0, 15.0);
      angleMap.put(1.5, 22.0);
      angleMap.put(3.0, 30.0);
      angleMap.put(4.0, 40.0);
    }
  }
}
