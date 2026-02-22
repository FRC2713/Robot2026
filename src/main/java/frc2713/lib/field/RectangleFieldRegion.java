package frc2713.lib.field;

import edu.wpi.first.math.geometry.Translation2d;

/** A field region backed by an axis-aligned rectangle with intersection support. */
public class RectangleFieldRegion extends frc2713.lib.geometry.Rectangle2d implements FieldRegion {
  public RectangleFieldRegion(Translation2d corner1, Translation2d corner2) {
    super(corner1, corner2);
  }
}
