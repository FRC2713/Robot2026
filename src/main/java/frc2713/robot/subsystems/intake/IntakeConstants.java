package frc2713.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc2713.lib.drivers.CANDeviceId;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;
import frc2713.lib.util.LoggedTunableMeasure;

public final class IntakeConstants {

  public static final class Roller {

    public static TalonFXSubsystemConfig config = new TalonFXSubsystemConfig();

    static {
      config.name = "Intake Roller";
      config.talonCANID = new CANDeviceId(9); // Example CAN ID, replace with actual ID
      config.unitToRotorRatio = 1.0; // 1:1 ratio
      config.momentOfInertia = 0.001; // Low MOI for fast-spinning rollers
    }

    public static Voltage intakeVoltageDesired = Volts.of(5.0);
    public static Voltage outtakeVoltageDesired = Volts.of(-5.0);
  }

  public static final class Extension {

    public static TalonFXSubsystemConfig config = new TalonFXSubsystemConfig();

    static {
      config.name = "Intake Extension";
      config.talonCANID = new CANDeviceId(8); // Example CAN ID, replace with actual ID
      config.fxConfig.Slot0.kP = 65.0;
      config.fxConfig.Slot0.kI = 0.0;
      config.fxConfig.Slot0.kD = 15.0;
      config.fxConfig.Slot0.kS = 0.125;
      config.fxConfig.Slot0.kV = 0.11;
      config.fxConfig.Slot0.kA = 0.0;
      config.fxConfig.MotionMagic.MotionMagicCruiseVelocity = 4.0; // target crusing vel rps
      config.fxConfig.MotionMagic.MotionMagicAcceleration = 6.0;
      config.fxConfig.MotionMagic.MotionMagicJerk = 0;

      config.unitToRotorRatio = 1.0; // assumes 1:1 gearbox
      config.unitRotationsPerMeter =
          1.0 / Inches.of(12).in(Meters); // assumes 1 ft of travel per rotation

      // Moment of inertia for sim - reasonable for light linear mechanism
      config.momentOfInertia = 0.01;
      config.tunable = true;
    }

    public static LoggedTunableMeasure<Distance> extendedPosition =
        new LoggedTunableMeasure<>(config.name + "/Extended Position", Inches.of(12.0));
    public static Distance retractedPosition = Inches.of(0);
    public static int MODEL_INDEX = 1;
    public static int PARENT_INDEX = 0; // drivetrain
  }
}
