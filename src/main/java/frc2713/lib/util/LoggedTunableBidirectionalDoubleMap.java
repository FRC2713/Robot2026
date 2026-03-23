package frc2713.lib.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringPublisher;
import frc2713.robot.Constants;

/**
 * A tunable version of BidirectionalInterpolatingDoubleMap. It reads key-value pairs from
 * NetworkTables (SmartDashboard compatible) and actively updates its internal map when in tuning
 * mode.
 */
public class LoggedTunableBidirectionalDoubleMap extends BidirectionalInterpolatingDoubleMap {
  private static final String TABLE_KEY = "Tuning/Maps/";

  private final StringEntry pointsEntry;
  private final StringPublisher pointsPublisher;

  private String lastHasChangedString = "";
  private double keyMultiplier = 1.0;
  private double valueMultiplier = 1.0;

  /**
   * Create a new LoggedTunableBidirectionalDoubleMap
   *
   * @param name Name of the map for the dashboard
   */
  public LoggedTunableBidirectionalDoubleMap(String name) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(TABLE_KEY + name);
    pointsEntry = table.getStringTopic("Points").getEntry("");
    pointsPublisher = table.getStringTopic("Points").publish();
  }

  /**
   * Set multipliers for the Dashboard string values to convert back to the native units used by the
   * map. Useful to show meters on dashboard, but use feet in the Map interpolation.
   *
   * @param keyMultiplier Multiply the dashboard key by this value when parsing from
   *     Dashboard/String. Divide the map key by this when writing to the Dashboard.
   * @param valueMultiplier Multiply the dashboard value by this value when parsing from
   *     Dashboard/String. Divide the map value by this when writing to the Dashboard.
   */
  public void setDashboardMultipliers(double keyMultiplier, double valueMultiplier) {
    this.keyMultiplier = keyMultiplier;
    this.valueMultiplier = valueMultiplier;
  }

  /**
   * Populates the map with an initial set of points and pushes them to the dashboard.
   *
   * @param keys Initial keys
   * @param values Initial values
   */
  public void initDefault(double[] keys, double[] values) {
    if (keys.length != values.length) {
      throw new IllegalArgumentException("Keys and values arrays must be the same length.");
    }

    super.clear();
    StringBuilder sb = new StringBuilder();
    for (int i = 0; i < keys.length; i++) {
      super.put(keys[i], values[i]);
      sb.append("{")
          .append(keys[i] / keyMultiplier)
          .append(", ")
          .append(values[i] / valueMultiplier)
          .append("}");
      if (i < keys.length - 1) {
        sb.append(", ");
      }
    }

    if (Constants.tuningMode) {
      pointsPublisher.set(sb.toString());
    }
  }

  /**
   * Initializes the map using a 2D array of key-value pairs, which reads cleanly like a dictionary.
   *
   * @param pairs A 2D array where each element is a double array of {key, value}
   */
  public void initDefault(double[][] pairs) {
    double[] keys = new double[pairs.length];
    double[] values = new double[pairs.length];
    for (int i = 0; i < pairs.length; i++) {
      if (pairs[i].length != 2)
        throw new IllegalArgumentException(
            "Each pair must have exactly two elements (key and value).");
      keys[i] = pairs[i][0];
      values[i] = pairs[i][1];
    }
    initDefault(keys, values);
  }

  /**
   * Checks if the map data was changed from the dashboard and updates internal state if necessary.
   */
  public void updateFromDashboard() {
    if (!Constants.tuningMode) {
      return;
    }

    String currentString = pointsEntry.get("");

    if (!currentString.equals(lastHasChangedString)) {
      try {
        // Parse strings like "{1.0, 10.0}, {2.0, 20.0}"
        String[] pairs = currentString.split("},\\s*\\{");

        super.clear();
        for (String pair : pairs) {
          // Remove all leftover curly braces and spaces
          String cleaned = pair.replaceAll("[{}\\s]", "");
          if (cleaned.isEmpty()) continue;

          String[] split = cleaned.split(",");
          if (split.length == 2) {
            double key = Double.parseDouble(split[0]) * keyMultiplier;
            double value = Double.parseDouble(split[1]) * valueMultiplier;
            super.put(key, value);
          }
        }
        lastHasChangedString = currentString;
      } catch (Exception e) {
        // Ignore parsing errors so it doesn't crash the robot when typing
      }
    }
  }

  @Override
  public Double get(Double key) {
    updateFromDashboard();
    return super.get(key);
  }

  @Override
  public double reverseGet(double output) {
    updateFromDashboard();
    return super.reverseGet(output);
  }
}
