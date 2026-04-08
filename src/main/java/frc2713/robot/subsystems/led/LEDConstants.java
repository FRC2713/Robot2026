package frc2713.robot.subsystems.led;

import frc2713.lib.drivers.CANDeviceId;

public final class LEDConstants {
  private LEDConstants() {}

  /** CANdle CAN ID and bus. Update these values to match your robot wiring. */
  public static final CANDeviceId candleCanId = new CANDeviceId(50, "canivore");

  /** Number of LEDs in the primary strip wired to the CANdle. */
  public static final int ledCount = 74;

  /** Number of animation slots supported by this Phoenix API generation. */
  public static final int animationSlotCount = 8;

  /** Startup/default color (orange). */
  public static final int defaultRed = 255;

  public static final int defaultGreen = 70;
  public static final int defaultBlue = 0;

  /** Match time threshold (seconds) for endgame panic LED behavior during teleop. */
  public static final double endgamePanicThresholdSec = 30.0;

  /** State colors. */
  public static final int problemRed = 255;

  public static final int problemGreen = 0;
  public static final int problemBlue = 0;

  public static final int stopRed = 255;
  public static final int stopGreen = 0;
  public static final int stopBlue = 255;

  public static final int launchingRed = 0;
  public static final int launchingGreen = 255;
  public static final int launchingBlue = 0;

  public static final int intakingRed = 255;
  public static final int intakingGreen = 180;
  public static final int intakingBlue = 0;

  public static final int detectingRed = 0;
  public static final int detectingGreen = 120;
  public static final int detectingBlue = 255;
}
