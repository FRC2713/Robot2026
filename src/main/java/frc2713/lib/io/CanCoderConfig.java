package frc2713.lib.io;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import frc2713.lib.drivers.CANDeviceId;

/** Configuration for a CANcoder. */
public class CanCoderConfig {
  public CANDeviceId canId;
  public CANcoderConfiguration config = new CANcoderConfiguration();

  public CanCoderConfig() {}

  public CanCoderConfig(CANDeviceId canId) {
    this.canId = canId;
  }
}
