package frc2713.lib.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A field region backed by a rectangle (axis-aligned or rotated).
 *
 * <p>Supports point containment ({@link #contains}) and rectangle intersection ({@link
 * #intersects}) via the {@link FieldRegion} interface. All coordinates are in meters.
 *
 * <p>Typically used for field zones such as the neutral zone or alliance zone in {@link
 * frc2713.robot.FieldConstants}.
 */
public class RectangleFieldRegion extends frc2713.lib.geometry.Rectangle2d implements FieldRegion {

  /**
   * Constructs an axis-aligned rectangular field region from two diagonally opposite corners.
   *
   * @param corner1 First corner of the rectangle, in meters
   * @param corner2 Second corner (diagonal from corner1), in meters
   */
  public RectangleFieldRegion(Translation2d corner1, Translation2d corner2) {
    super(corner1, corner2);
  }

  /**
   * Constructs a rectangular field region from its center pose and dimensions. Supports rotated
   * rectangles via the rotation component of the pose.
   *
   * @param center Center position and rotation of the rectangle, in meters and radians
   * @param xWidth Width along the local x-axis (before rotation), in meters
   * @param yWidth Height along the local y-axis (before rotation), in meters
   */
  public RectangleFieldRegion(Pose2d center, double xWidth, double yWidth) {
    super(center, xWidth, yWidth);
  }
}
