package frc2713.lib.util;

import java.util.HashMap;

public final class CachedPow {

  private final double exp;
  private final double resolution;
  private final HashMap<Long, Double> cache;

  /**
   * every bit helps(?)
   *
   * @param exp the fixed fractional exponent
   * @param resolution defines b
   */
  public CachedPow(double exp, double resolution) {
    this.exp = exp;
    this.resolution = resolution;
    this.cache = new HashMap<>();
  }

  public CachedPow(double exp) {
    this(exp, 0.001);
  }

  public double get(double input) {
    // This effectively buckets together similar values for the cache
    long key = (long) (input / resolution);

    // Cache lookup - O(1) with hash maps
    Double cachedResult = cache.get(key);
    if (cachedResult != null) {
      return cachedResult;
    }

    // Cache miss - compute new value
    double result = Math.pow(input, exp);
    cache.put(key, result);

    return result;
  }

  public void reset() {
    cache.clear();
  }
}
