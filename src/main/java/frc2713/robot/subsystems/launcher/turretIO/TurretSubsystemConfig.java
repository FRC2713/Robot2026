package frc2713.robot.subsystems.launcher.turretIO;

import frc2713.lib.drivers.CANDeviceId;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;

/** Configuration for the Turret subsystem with CANcoder support. */
public class TurretSubsystemConfig extends TalonFXSubsystemConfig {

  /** CAN ID for the external CANcoder used as encoder 2. */
  public CANDeviceId canCoderCANID;
}
