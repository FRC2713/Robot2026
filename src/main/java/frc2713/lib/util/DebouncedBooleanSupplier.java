package frc2713.lib.util;

import edu.wpi.first.wpilibj.Timer;
import java.util.function.BooleanSupplier;

/**
 * A BooleanSupplier wrapper that debounces boolean signals on both rising and falling edges. Useful
 * for preventing jittery behavior when signals flicker rapidly.
 *
 * <p>Example usage:
 *
 * <pre>
 * DebouncedBooleanSupplier debounced = new DebouncedBooleanSupplier(
 *     () -> sensor.isTriggered(),
 *     0.5,  // Rising edge: signal must be true for 0.5s before output becomes true
 *     0.1   // Falling edge: signal must be false for 0.1s before output becomes false
 * );
 *
 * // In periodic():
 * debounced.update();
 * boolean stableValue = debounced.getAsBoolean();
 * </pre>
 */
public class DebouncedBooleanSupplier implements BooleanSupplier {
  private final BooleanSupplier rawSignal;
  private final double risingEdgeDebounce;
  private final double fallingEdgeDebounce;

  private boolean debouncedState = false;
  private double transitionStartTime = 0.0;
  private boolean lastRawSignal = false;

  /**
   * Creates a new DebouncedBooleanSupplier.
   *
   * @param rawSignal the raw boolean signal to debounce
   * @param risingEdgeDebounce seconds the signal must be true before the debounced output becomes
   *     true
   * @param fallingEdgeDebounce seconds the signal must be false before the debounced output becomes
   *     false
   */
  public DebouncedBooleanSupplier(
      BooleanSupplier rawSignal, double risingEdgeDebounce, double fallingEdgeDebounce) {
    this.rawSignal = rawSignal;
    this.risingEdgeDebounce = risingEdgeDebounce;
    this.fallingEdgeDebounce = fallingEdgeDebounce;
  }

  /**
   * Updates the debounced state. This must be called periodically (typically in a subsystem's
   * periodic() method or in a command's execute() method).
   */
  public void update() {
    boolean currentRawSignal = rawSignal.getAsBoolean();
    double currentTime = Timer.getFPGATimestamp();

    // Detect edge transitions
    if (currentRawSignal != lastRawSignal) {
      transitionStartTime = currentTime;
      lastRawSignal = currentRawSignal;
    }

    // Check if debounce time has elapsed
    double timeInCurrentState = currentTime - transitionStartTime;
    if (currentRawSignal && !debouncedState) {
      // Rising edge: raw signal is true, debounced is false
      if (timeInCurrentState >= risingEdgeDebounce) {
        debouncedState = true;
      }
    } else if (!currentRawSignal && debouncedState) {
      // Falling edge: raw signal is false, debounced is true
      if (timeInCurrentState >= fallingEdgeDebounce) {
        debouncedState = false;
      }
    }
  }

  /**
   * Gets the debounced boolean value.
   *
   * @return the debounced state (only changes after debounce time has elapsed)
   */
  @Override
  public boolean getAsBoolean() {
    return debouncedState;
  }

  /**
   * Gets the raw (non-debounced) boolean value.
   *
   * @return the current raw signal value
   */
  public boolean getRaw() {
    return rawSignal.getAsBoolean();
  }

  /** Resets the debouncer to its initial state (false output, no transition in progress). */
  public void reset() {
    debouncedState = false;
    transitionStartTime = 0.0;
    lastRawSignal = false;
  }
}
