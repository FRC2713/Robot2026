package frc2713.robot.subsystems.launcher;

import frc2713.lib.drivers.CANDeviceId;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;

/** Configuration for the Turret subsystem, extending base TalonFX config with CANCoder support. */
public class TurretSubsystemConfig extends TalonFXSubsystemConfig {

  /** CAN ID for the external CANCoder used as encoder 2 */
  public CANDeviceId canCoderCANID;
}
