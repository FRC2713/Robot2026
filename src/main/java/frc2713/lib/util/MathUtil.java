package frc2713.lib.util;

public class MathUtil {

  /**
   * Fast integer exponentiation using exponentiation by squaring. O(log exp) multiplications.
   * Handles negative exponents (returns 0 for int). Also see {@link MathUtil.sq(long)}
   *
   * @param base the base
   * @param exp the exponent (non-negative)
   * @return base^exp as a long
   */
  public static long powInt(long base, int exp) {
    if (exp < 0) return 0; // integers can't represent fractions
    if (exp == 0) return 1;
    long result = 1;
    while (exp > 0) {
      if ((exp & 1) == 1) result *= base; // if current bit is set, multiply in base
      base *= base;
      exp >>= 1; // shift to next bit
    }
    return result;
  }

  /**
   * Fast double exponentiation by squaring. Faster than Math.pow for integer exponents; use
   * Math.pow for fractional ones. Also see {@link MathUtil.sq(double)}
   *
   * @param base the base
   * @param exp the exponent (any integer, including negative)
   * @return base^exp as a double
   */
  public static double powDouble(double base, int exp) {
    if (exp == 0) return 1.0;
    if (exp < 0) {
      base = 1.0 / base;
      exp = -exp;
    }
    double result = 1.0;
    while (exp > 0) {
      if ((exp & 1) == 1) result *= base;
      base *= base;
      exp >>= 1;
    }
    return result;
  }

  /** Squares a value. Avoids Math.pow(x, 2) overhead entirely. */
  public static double sq(double x) {
    return x * x;
  }

  public static float sq(float x) {
    return x * x;
  }

  public static long sq(long x) {
    return x * x;
  }

  public static int sq(int x) {
    return x * x;
  }
}
