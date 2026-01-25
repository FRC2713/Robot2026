package frc2713.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import frc2713.lib.drivers.CANDeviceId;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;

public final class LauncherConstants {

  public static final class Turret {

    public static TalonFXSubsystemConfig config = new TalonFXSubsystemConfig();

    static {
      config.name = "Turret";
      config.talonCANID = new CANDeviceId(12); // Example CAN ID, replace with actual ID
      config.fxConfig.Slot0.kP = 0.2;
      config.fxConfig.Slot0.kI = 0.0;
      config.fxConfig.Slot0.kD = 0.0;
      config.unitToRotorRatio = 1.0; // 1:1 ratio
      config.initialTransform =
          new Transform3d(
              new Translation3d(0, Inches.of(1.75).in(Meters), Inches.of(18.484119).in(Meters)),
              new Rotation3d(0, 0, 0));
    }

    public static int MODEL_INDEX = 3;
    public static int PARENT_INDEX = 0; // drivetrain
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
  }
}
