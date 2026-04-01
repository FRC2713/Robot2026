package frc2713.lib.util;

import java.util.HashMap;

public final class CachedPow {

  private final double exp;
  private final double bucketSize;
  private final HashMap<Long, Double> cache;

  /**
   * every bit helps(?)
   *
   * @param exp the fixed fractional exponent
   * @param inputBucketSize defines how floating point entries are stored in cache
   */
  public CachedPow(double exp, double inputBucketSize) {
    this.exp = exp;
    this.bucketSize = inputBucketSize;
    this.cache = new HashMap<>();
  }

  public CachedPow(double exp) {
    this(exp, 0.001);
  }

  public double get(double input) {
    // This buckets together similar values for the cache
    long key = (long) (input / bucketSize);

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
