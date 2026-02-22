package frc2713.lib.field;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Supplier;

/** A field region for containment checks. All coordinates are in meters (SI). */
public interface FieldRegion {
  /** Returns true if the given point (in meters) is inside this region. */
  boolean contains(Translation2d point);

  /** Returns true of the given rectangle intersects with this region. */
  boolean intersects(Rectangle2d rectangle);

  /** Creates a Trigger that activates when the supplied position is inside this region. */
  default Trigger createContainsTrigger(Supplier<Translation2d> positionSupplier) {
    return new Trigger(() -> contains(positionSupplier.get()));
  }

  /** Creates a Trigger that activates when the supplied rectangle intersects with this region. */
  default Trigger createIntersectsTrigger(Supplier<Rectangle2d> rectangleSupplier) {
    return new Trigger(() -> intersects(rectangleSupplier.get()));
  }
}
