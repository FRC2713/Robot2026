package frc2713.robot.subsystems.led;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import frc2713.robot.util.PhoenixUtil;

public class LEDIOCANdle implements LEDIO {
  protected final CANdle candle;

  protected int red = LEDConstants.defaultRed;
  protected int green = LEDConstants.defaultGreen;
  protected int blue = LEDConstants.defaultBlue;

  public LEDIOCANdle() {
    candle =
        new CANdle(
            LEDConstants.candleCanId.getDeviceNumber(),
            new CANBus(LEDConstants.candleCanId.getBus()));
    candle.optimizeBusUtilization();
    setSolidColor(red, green, blue);
  }

  @Override
  public void updateInputs(LEDInputs inputs) {
    inputs.connected = candle.isConnected();
    inputs.usingSimState = false;
    inputs.red = red;
    inputs.green = green;
    inputs.blue = blue;
  }

  @Override
  public StatusCode setSolidColor(int red, int green, int blue) {
    this.red = clampColor(red);
    this.green = clampColor(green);
    this.blue = clampColor(blue);

    StatusCode clearStatus = clearAnimations();

    StatusCode[] status = new StatusCode[] {StatusCode.OK};
    int endLed = Math.max(0, LEDConstants.ledCount - 1);
    RGBWColor color = new RGBWColor(this.red, this.green, this.blue);
    PhoenixUtil.tryUntilOk(
        3,
        () -> {
          status[0] =
              candle.setControl(new SolidColor(0, endLed).withColor(color).withUpdateFreqHz(0));
          return status[0];
        });

    return status[0].isOK() ? clearStatus : status[0];
  }

  @Override
  public StatusCode clearAnimations() {
    StatusCode status = StatusCode.OK;
    for (int slot = 0; slot < LEDConstants.animationSlotCount; slot++) {
      int animationSlot = slot;
      StatusCode[] slotStatus = new StatusCode[] {StatusCode.OK};
      PhoenixUtil.tryUntilOk(
          3,
          () -> {
            slotStatus[0] = candle.setControl(new EmptyAnimation(animationSlot));
            return slotStatus[0];
          });

      if (!slotStatus[0].isOK()) {
        status = slotStatus[0];
      }
    }

    return status;
  }

  @Override
  public StatusCode setControl(ControlRequest request) {
    return candle.setControl(request);
  }

  protected static int clampColor(int value) {
    return Math.max(0, Math.min(255, value));
  }
}
