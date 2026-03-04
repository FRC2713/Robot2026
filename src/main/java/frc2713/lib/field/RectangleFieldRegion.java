package frc2713.lib.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

  /**
   * Constructs a rectangular field region from min/max X and Y bounds.
   *
   * @param minX Minimum X coordinate (meters)
   * @param maxX Maximum X coordinate (meters)
   * @param minY Minimum Y coordinate (meters)
   * @param maxY Maximum Y coordinate (meters)
   */
  public RectangleFieldRegion(double minX, double maxX, double minY, double maxY) {
    super(new Translation2d(minX, minY), new Translation2d(maxX, maxY));
  }

  @Override
  public Pose2d[] getBoundaryPoses() {
    Pose2d center = getCenter();
    double halfWidth = getXWidth() / 2;
    double halfHeight = getYWidth() / 2;
    Rotation2d rotation = center.getRotation();
    Rotation2d zero = new Rotation2d();

    if (rotation.equals(Rotation2d.kZero)) {
      // Axis-aligned: simple corner calculation
      double minX = center.getX() - halfWidth;
      double maxX = center.getX() + halfWidth;
      double minY = center.getY() - halfHeight;
      double maxY = center.getY() + halfHeight;

      return new Pose2d[] {
        new Pose2d(minX, minY, zero), // Bottom-left
        new Pose2d(maxX, minY, zero), // Bottom-right
        new Pose2d(maxX, maxY, zero), // Top-right
        new Pose2d(minX, maxY, zero), // Top-left
        new Pose2d(minX, minY, zero) // Close the polygon
      };
    } else {
      // Rotated rectangle: calculate corners using rotation
      Translation2d[] offsets = {
        new Translation2d(-halfWidth, -halfHeight),
        new Translation2d(halfWidth, -halfHeight),
        new Translation2d(halfWidth, halfHeight),
        new Translation2d(-halfWidth, halfHeight),
        new Translation2d(-halfWidth, -halfHeight) // Close polygon
      };

      Pose2d[] poses = new Pose2d[5];
      for (int i = 0; i < 5; i++) {
        Translation2d rotated = offsets[i].rotateBy(rotation);
        poses[i] = new Pose2d(center.getX() + rotated.getX(), center.getY() + rotated.getY(), zero);
      }
      return poses;
    }
  }
}
