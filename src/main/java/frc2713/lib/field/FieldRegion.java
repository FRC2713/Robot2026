package frc2713.lib.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * A field region for containment and intersection checks.
 *
 * <p>All coordinates are in meters (SI). Implementations define regions on the field (e.g. neutral
 * zone, alliance zone) and support point containment and rectangle intersection queries. Used with
 * {@link #createContainsTrigger} and {@link #createIntersectsTrigger} for command-based triggers.
 */
public interface FieldRegion {

  /**
   * Returns true if the given point is inside this region.
   *
   * @param point The point to check, in meters
   * @return true if the point is inside or on the boundary of this region
   */
  boolean contains(Translation2d point);

  /**
   * Returns true if the given rectangle intersects with this region (any overlap, including edges
   * and corners).
   *
   * @param rectangle The rectangle to check, in meters
   * @return true if the rectangle overlaps this region
   */
  boolean intersects(Rectangle2d rectangle);

  /**
   * Returns true if the given rectangle is entirely contained within this region.
   *
   * <p>All four corners and edges of the other rectangle must lie inside or on the boundary of this
   * region.
   *
   * @param rectangle The rectangle to check for containment, in meters
   * @return true if the rectangle is fully inside this region
   */
  boolean contains(Rectangle2d rectangle);

  /**
   * Creates a Trigger that activates when the supplied position is inside this region.
   *
   * @param positionSupplier Supplies the current position to check (e.g. robot pose translation)
   * @return A trigger that is active when the position is inside this region
   */
  default Trigger createContainsTrigger(Supplier<Translation2d> positionSupplier) {
    return new Trigger(() -> contains(positionSupplier.get()));
  }

  /**
   * Creates a Trigger that activates when the supplied rectangle is fully contained within this
   * region.
   *
   * @param rectangleSupplier Supplies the rectangle to check (e.g. robot footprint)
   * @return A trigger that is active when the rectangle is fully inside this region
   */
  default Trigger createContainsRectTrigger(Supplier<Rectangle2d> rectangleSupplier) {
    return new Trigger(() -> contains(rectangleSupplier.get()));
  }

  /**
   * Creates a Trigger that activates when the supplied rectangle intersects with this region.
   *
   * @param rectangleSupplier Supplies the rectangle to check (e.g. robot footprint)
   * @return A trigger that is active when the rectangle overlaps this region
   */
  default Trigger createIntersectsTrigger(Supplier<Rectangle2d> rectangleSupplier) {
    return new Trigger(() -> intersects(rectangleSupplier.get()));
  }

  // ========== VISUALIZATION METHODS ==========

  /**
   * Returns an array of Pose2d representing the boundary of this region for visualization in
   * AdvantageScope. For rectangular regions, returns the four corners plus a closing point. The
   * poses form a closed polygon when connected in order.
   *
   * @return Array of poses representing the region boundary (closed polygon)
   */
  default Pose2d[] getBoundaryPoses() {
    return new Pose2d[0]; // Override in implementations
  }

  /**
   * Returns a double array [x1, y1, x2, y2, ...] representing the region boundary for
   * visualization. This is a more efficient format for logging multiple regions.
   *
   * @return Array of x,y coordinate pairs forming a closed polygon
   */
  default double[] getBoundaryPoints() {
    Pose2d[] poses = getBoundaryPoses();
    double[] points = new double[poses.length * 2];
    for (int i = 0; i < poses.length; i++) {
      points[i * 2] = poses[i].getX();
      points[i * 2 + 1] = poses[i].getY();
    }
    return points;
  }

  /**
   * Logs this region to AdvantageScope for visualization on the field view.
   *
   * @param path The log path (e.g. "FieldRegions/MyZone")
   */
  default void log(String path) {
    Logger.recordOutput(path, getBoundaryPoses());
  }

  /**
   * Logs multiple regions to AdvantageScope as a combined array for visualization.
   *
   * @param path The log path (e.g. "FieldRegions/AllZones")
   * @param regions The regions to log
   */
  static void logAll(String path, FieldRegion... regions) {
    int totalPoses = 0;
    for (FieldRegion region : regions) {
      totalPoses += region.getBoundaryPoses().length;
    }

    Pose2d[] allPoses = new Pose2d[totalPoses];
    int index = 0;
    for (FieldRegion region : regions) {
      Pose2d[] poses = region.getBoundaryPoses();
      System.arraycopy(poses, 0, allPoses, index, poses.length);
      index += poses.length;
    }

    Logger.recordOutput(path, allPoses);
  }
}
