package frc2713.robot.subsystems.serializer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import frc2713.lib.drivers.CANDeviceId;
import frc2713.lib.dynamics.MoiUnits;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;
import frc2713.lib.util.LoggedTunableMeasure;
import frc2713.lib.util.Util;

public final class SerializerConstants {

  public static final class DyeRotor {

    public static TalonFXSubsystemConfig config = new TalonFXSubsystemConfig();
    public static double gearRatio = (44.0 / 8.0) * (114.0 / 12.0);

    public static final InterpolatingDoubleTreeMap otfSpeeds = new InterpolatingDoubleTreeMap();

    static {
      // Distance (m), RPM
      otfSpeeds.put(2.11, 80.); // north shore: 80
      otfSpeeds.put(3.2, 80.);
      otfSpeeds.put(4.2, 60.);
      otfSpeeds.put(6.44, 50.); // north shore: 30
    }

    static {
      config.name = "Dye Rotor";
      config.talonCANID = new CANDeviceId(45, "canivore"); // Example CAN ID, replace with actual ID

      config.useFOC = false;

      // Velocity PID gains for VelocityVoltage control
      // Units: kP/kV/kS are in volts
      config.fxConfig.Slot0.kP =
          Util.modeDependentValue(15., 0.0); // Volts per rps of error (lower to reduce oscillation)
      config.fxConfig.Slot0.kI = 0.0;
      config.fxConfig.Slot0.kD = 0.0; // Small damping to reduce oscillation
      config.fxConfig.Slot0.kS =
          Util.modeDependentValue(0.275, 0.); // Volts to overcome static friction
      config.fxConfig.Slot0.kV = 0.12 * gearRatio; // Volts per rps (feedforward)

      config.fxConfig.CurrentLimits =
          new CurrentLimitsConfigs()
              .withStatorCurrentLimit(Amps.of(80))
              .withStatorCurrentLimitEnable(true);

      config.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      config.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      config.unitToRotorRatio = gearRatio;

      config.momentOfInertia = MoiUnits.PoundSquareInches.of(89.2780792);

      config.initialTransform =
          new Transform3d(
              new Translation3d(Inches.of(0.5).in(Meters), Inches.of(0.5).in(Meters), 0),
              new Rotation3d());

      config.simOrientation = ChassisReference.CounterClockwise_Positive;
    }

    public static LoggedTunableMeasure<AngularVelocity> indexingSpeed =
        new LoggedTunableMeasure<AngularVelocity>("Dye Rotor/Index Speed", RPM.of(65));
    public static LoggedTunableMeasure<AngularVelocity> stirSpeed =
        new LoggedTunableMeasure<AngularVelocity>("Dye Rotor/Stir Speed", RPM.of(30));
    public static AngularVelocity outdexingSpeed = RotationsPerSecond.of(-3);

    public static int MODEL_INDEX = 2;
    public static int PARENT_INDEX = 0; // drivetrain
  }

  public static final class Feeder {
    public static TalonFXSubsystemConfig config = new TalonFXSubsystemConfig();
    public static double gearRatio = 37.0 / 14.0;

    static {
      config.name = "Feeder";
      config.talonCANID = new CANDeviceId(44, "canivore"); // Example CAN ID, replace with actual ID
      config.useFOC = false;
      config.motor = DCMotor.getKrakenX60(1);

      // Velocity PID gains for VelocityVoltage control
      // Units: kP/kV/kS are in volts
      config.fxConfig.Slot0.kP = Util.modeDependentValue(0.5, 0.11); // Volts per rps of error
      config.fxConfig.Slot0.kI = 0.0;
      config.fxConfig.Slot0.kD = 0.0;
      config.fxConfig.Slot0.kS =
          Util.modeDependentValue(0.275, 0.1); // Volts to overcome static friction
      config.fxConfig.Slot0.kV = 0.12 * gearRatio; // Volts per rps (feedforward)

      config.fxConfig.CurrentLimits =
          new CurrentLimitsConfigs()
              .withStatorCurrentLimit(Amps.of(80))
              .withStatorCurrentLimitEnable(true);

      config.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      config.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      config.unitToRotorRatio = gearRatio;

      // tough to estimate this, but I just determined the MOI of the wheel assembly by the hook and
      // tripled it
      config.momentOfInertia = MoiUnits.PoundSquareInches.of(0.075 * 3);
    }

    public static AngularVelocity freeSpeed =
        RadiansPerSecond.of(config.motor.freeSpeedRadPerSec).div(gearRatio);

    public static LoggedTunableMeasure<AngularVelocity> shootingSpeed =
        new LoggedTunableMeasure<AngularVelocity>("Feeder/Speed", RPM.of(freeSpeed.in(RPM)));

    public static LoggedTunableMeasure<AngularVelocity> unjammingSpeed =
        new LoggedTunableMeasure<AngularVelocity>("Feeder/UnJamming Speed", RPM.of(2000));
  }
}
