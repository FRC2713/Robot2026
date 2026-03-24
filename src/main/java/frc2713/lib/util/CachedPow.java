package frc2713.lib.util;

public final class CachedPow {

  private final double exp;
  private double lastInput;
  private double lastResult;
  private final double threshold;
  private boolean hasValue;

  /**
   * every bit helps(?)
   *
   * @param exp the fixed fractional exponent
   * @param threshold only recompute if input changes by more than this amount
   */
  public CachedPow(double exp, double threshold) {
    this.exp = exp;
    this.threshold = threshold;
    this.hasValue = false;
  }

  public CachedPow(double exp) {
    this(exp, 0.001);
  }

  public double get(double input) {
    if (!hasValue || Math.abs(input - lastInput) > threshold) {
      lastInput = input;
      lastResult = Math.pow(input, exp);
      hasValue = true;
    }
    return lastResult;
  }

  public void reset() {
    hasValue = false;
  }
}
