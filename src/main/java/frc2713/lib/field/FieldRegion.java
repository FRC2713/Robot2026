package frc2713.lib.field;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Supplier;

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
   * Creates a Trigger that activates when the supplied position is inside this region.
   *
   * @param positionSupplier Supplies the current position to check (e.g. robot pose translation)
   * @return A trigger that is active when the position is inside this region
   */
  default Trigger createContainsTrigger(Supplier<Translation2d> positionSupplier) {
    return new Trigger(() -> contains(positionSupplier.get()));
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
}
