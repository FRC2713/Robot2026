package frc2713.robot.subsystems.serializer;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import frc2713.lib.drivers.CANDeviceId;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;
import frc2713.lib.util.LoggedTunableMeasure;

public final class SerializerConstants {

  public static final class DyeRotor {

    public static TalonFXSubsystemConfig config = new TalonFXSubsystemConfig();

    static {
      config.name = "Dye Rotor";
      config.talonCANID = new CANDeviceId(43); // Example CAN ID, replace with actual ID

      config.useFOC = false;

      // Velocity PID gains for VelocityVoltage control
      // Units: kP/kV/kS are in volts
      config.fxConfig.Slot0.kP = 0.08; // Volts per rps of error (lower to reduce oscillation)
      config.fxConfig.Slot0.kI = 0.0;
      config.fxConfig.Slot0.kD = 0.01; // Small damping to reduce oscillation
      config.fxConfig.Slot0.kS = 0.1; // Volts to overcome static friction
      config.fxConfig.Slot0.kV = 0.12; // Volts per rps (feedforward)

      config.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      config.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      config.unitToRotorRatio = 46.0 / 14.0;
      config.momentOfInertia = 0.001; // kg*m^2 for simulation (small roller)

      config.initialTransform =
          new Transform3d(new Translation3d(0, Inches.of(1.75).in(Meters), 0), new Rotation3d());
    }

    public static LoggedTunableMeasure<AngularVelocity> indexingSpeed =
        new LoggedTunableMeasure<AngularVelocity>("DyeRotor/Speed", RotationsPerSecond.of(60));
    public static AngularVelocity outdexingSpeed = RotationsPerSecond.of(-3);

    public static int MODEL_INDEX = 2;
    public static int PARENT_INDEX = 0; // drivetrain
  }

  public static final class Feeder {

    public static TalonFXSubsystemConfig config = new TalonFXSubsystemConfig();

    static {
      config.name = "Feeder";
      config.talonCANID = new CANDeviceId(44); // Example CAN ID, replace with actual ID

      config.useFOC = false;

      // Velocity PID gains for VelocityVoltage control
      // Units: kP/kV/kS are in volts
      config.fxConfig.Slot0.kP = 0.11; // Volts per rps of error
      config.fxConfig.Slot0.kI = 0.0;
      config.fxConfig.Slot0.kD = 0.0;
      config.fxConfig.Slot0.kS = 0.1; // Volts to overcome static friction
      config.fxConfig.Slot0.kV = 0.12; // Volts per rps (feedforward)

      config.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      config.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      config.unitToRotorRatio = 1.0;
      config.momentOfInertia = 0.001; // kg*m^2 for simulation
    }

    public static LoggedTunableMeasure<AngularVelocity> shootingSpeed =
        new LoggedTunableMeasure<AngularVelocity>("Feeder/Speed", RotationsPerSecond.of(100));
  }
}
