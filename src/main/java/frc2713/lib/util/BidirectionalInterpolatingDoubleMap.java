package frc2713.lib.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class BidirectionalInterpolatingDoubleMap extends InterpolatingDoubleTreeMap {
  private final InterpolatingDoubleTreeMap reverse = new InterpolatingDoubleTreeMap();

  @Override
  public void put(Double key, Double value) {
    super.put(key, value);
    reverse.put(value, key);
  }

  @Override
  public void clear() {
    super.clear();
    reverse.clear();
  }

  public double reverseGet(double output) {
    return reverse.get(output);
  }
}
