package frc2713.robot.subsystems.serializer;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import frc2713.lib.drivers.CANDeviceId;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;

public final class SerializerConstants {

  public static final class DyeRotor {

    public static TalonFXSubsystemConfig config = new TalonFXSubsystemConfig();

    static {
      config.name = "Dye Rotor";
      config.talonCANID = new CANDeviceId(10); // Example CAN ID, replace with actual ID

      // Velocity PID gains
      config.fxConfig.Slot0.kP = 0.3;
      config.fxConfig.Slot0.kI = 0.0;
      config.fxConfig.Slot0.kD = 0.0;
      config.fxConfig.Slot0.kS = 0.1; // Static friction compensation
      config.fxConfig.Slot0.kV = 0.12; // Velocity feedforward

      config.unitToRotorRatio = 1.0; // 1:1 ratio
      config.momentOfInertia = 0.001; // kg*m^2 for simulation (small roller)

      config.initialTransform =
          new Transform3d(new Translation3d(0, Inches.of(1.75).in(Meters), 0), new Rotation3d());
    }

    public static AngularVelocity indexingSpeed = RotationsPerSecond.of(-3);
    public static AngularVelocity outdexingSpeed = RotationsPerSecond.of(3);

    public static int MODEL_INDEX = 2;
    public static int PARENT_INDEX = 0; // drivetrain
  }

  public static final class Feeder {

    public static TalonFXSubsystemConfig config = new TalonFXSubsystemConfig();

    static {
      config.name = "Feeder";
      config.talonCANID = new CANDeviceId(11); // Example CAN ID, replace with actual ID
      config.fxConfig.Slot0.kP = 0.2;
      config.fxConfig.Slot0.kI = 0.0;
      config.fxConfig.Slot0.kD = 0.0;
      config.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      config.unitToRotorRatio = 1.0; // 1:1 ratio
      config.momentOfInertia = 0.001; // kg*m^2 for simulation
    }

    public static AngularVelocity shootingSpeed = RotationsPerSecond.of(30);
  }
}
