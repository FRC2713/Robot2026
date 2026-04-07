package frc2713.robot.subsystems.led;

import edu.wpi.first.wpilibj.RobotController;

/**
 * Sim IO implementation for CANdle-based LEDs.
 *
 * <p>Uses Phoenix CANdle sim state to publish supply telemetry while reusing normal CANdle control
 * requests for colors and animations.
 */
public class LEDIOSim extends LEDIOCANdle {

  @Override
  public void updateInputs(LEDInputs inputs) {
    candle.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

    inputs.connected = true;
    inputs.usingSimState = true;
    inputs.red = red;
    inputs.green = green;
    inputs.blue = blue;
  }
}
