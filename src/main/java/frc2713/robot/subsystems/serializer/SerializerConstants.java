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
import frc2713.lib.dynamics.MoiUnits;
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

      config.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      config.unitToRotorRatio =
          (8.0 / 44.0)
              * (12.0
                  / 114.0); // 8 tooth pinion to 44 tooth gear, 12 tooth gear to 114 tooth gear for
      // total reduction of 0.0195
      config.momentOfInertia = MoiUnits.PoundSquareInches.of(89.2780792);

      config.initialTransform =
          new Transform3d(new Translation3d(0, Inches.of(1.75).in(Meters), 0), new Rotation3d());
    }

    public static LoggedTunableMeasure<AngularVelocity> indexingSpeed =
        new LoggedTunableMeasure<AngularVelocity>("DyeRotor/Speed", RotationsPerSecond.of(-3));
    public static AngularVelocity outdexingSpeed = RotationsPerSecond.of(3);

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

      config.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      config.unitToRotorRatio =
          14.0 / 37.0; // 14 tooth gear on motor to 37 tooth gear on feeder roller
      // tough to estimate this, but I just determined the MOI of the wheel assembly by the hook and
      // tripled it
      config.momentOfInertia = MoiUnits.PoundSquareInches.of(0.075 * 3);
    }

    public static LoggedTunableMeasure<AngularVelocity> shootingSpeed =
        new LoggedTunableMeasure<AngularVelocity>("Feeder/Speed", RotationsPerSecond.of(30));
  }
}
