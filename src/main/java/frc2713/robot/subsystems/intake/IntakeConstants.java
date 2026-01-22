package frc2713.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import frc2713.lib.drivers.CANDeviceId;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;

public final class IntakeConstants {

  public static final class Roller {

    public static TalonFXSubsystemConfig config = new TalonFXSubsystemConfig();

    static {
      config.name = "Intake Roller";
      config.talonCANID = new CANDeviceId(9); // Example CAN ID, replace with actual ID
      config.unitToRotorRatio = 1.0; // 1:1 ratio
    }

    public static double intakeDutyCycle = 1.0;
    public static double outtakeDutyCycle = -1.0;
  }

  public static final class Extension {

    public static TalonFXSubsystemConfig config = new TalonFXSubsystemConfig();

    static {
      config.name = "Intake Extension";
      config.talonCANID = new CANDeviceId(8); // Example CAN ID, replace with actual ID
      config.fxConfig.Slot0.kP = 0.2;
      config.fxConfig.Slot0.kI = 0.0;
      config.fxConfig.Slot0.kD = 0.0;
      config.unitToRotorRatio = 1.0; // 1:1 ratio
      
    }

    public static Distance extendedPostion = Inches.of(12);
    public static Distance retractedPosition = Inches.of(0);
  }
}
