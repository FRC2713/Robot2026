package frc2713.robot.subsystems.led;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.ControlRequest;
import org.littletonrobotics.junction.AutoLog;

public interface LEDIO {
  @AutoLog
  public static class LEDInputs {
    public boolean connected = false;
    public boolean usingSimState = false;

    public int red = LEDConstants.defaultRed;
    public int green = LEDConstants.defaultGreen;
    public int blue = LEDConstants.defaultBlue;
  }

  public default void updateInputs(LEDInputs inputs) {}

  public default StatusCode setSolidColor(int red, int green, int blue) {
    return StatusCode.OK;
  }

  public default StatusCode clearAnimations() {
    return StatusCode.OK;
  }

  public default StatusCode setControl(ControlRequest request) {
    return StatusCode.OK;
  }
}
