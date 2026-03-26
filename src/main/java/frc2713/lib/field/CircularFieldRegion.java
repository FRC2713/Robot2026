package frc2713.lib.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/** A circular field region on a fixed z-plane. */
public class CircularFieldRegion implements FieldRegion {
  private static final int BOUNDARY_SEGMENTS = 32;

  private final Translation2d center;
  private final Translation3d center3d;
  private final double radiusMeters;
  private final double zMeters;

  public CircularFieldRegion(Translation2d center, double radiusMeters) {
    this(center, radiusMeters, 0.0);
  }

  public CircularFieldRegion(Translation2d center, double radiusMeters, double zMeters) {
    this.center = center;
    this.radiusMeters = radiusMeters;
    this.zMeters = zMeters;
    this.center3d = new Translation3d(center.getX(), center.getY(), zMeters);
  }

  @Override
  public boolean contains(Translation2d point) {
    return center.getDistance(point) <= radiusMeters;
  }

  @Override
  public boolean intersects(Rectangle2d rectangle) {
    Translation2d[] corners = getCorners(rectangle);
    for (Translation2d corner : corners) {
      if (contains(corner)) {
        return true;
      }
    }

    if (rectangle.contains(center)) {
      return true;
    }

    for (int i = 0; i < 4; i++) {
      Translation2d p1 = corners[i];
      Translation2d p2 = corners[(i + 1) % 4];
      if (distancePointToSegment(center, p1, p2) <= radiusMeters) {
        return true;
      }
    }
    return false;
  }

  @Override
  public boolean contains(Rectangle2d rectangle) {
    Translation2d[] corners = getCorners(rectangle);
    for (Translation2d corner : corners) {
      if (!contains(corner)) {
        return false;
      }
    }
    return true;
  }

  @Override
  public Pose2d[] getBoundaryPoses() {
    Pose2d[] poses = new Pose2d[BOUNDARY_SEGMENTS + 1];
    for (int i = 0; i <= BOUNDARY_SEGMENTS; i++) {
      double angle = 2.0 * Math.PI * i / BOUNDARY_SEGMENTS;
      double x = center.getX() + radiusMeters * Math.cos(angle);
      double y = center.getY() + radiusMeters * Math.sin(angle);
      poses[i] = new Pose2d(x, y, Rotation2d.kZero);
    }
    return poses;
  }

  @Override
  public double getZMeters() {
    return zMeters;
  }

  public Translation2d getCenter() {
    return center;
  }

  public double getRadiusMeters() {
    return radiusMeters;
  }

  public Translation3d getCenter3d() {
    return center3d;
  }

  private static Translation2d[] getCorners(Rectangle2d rectangle) {
    double halfX = rectangle.getXWidth() / 2.0;
    double halfY = rectangle.getYWidth() / 2.0;
    Pose2d centerPose = rectangle.getCenter();
    Rotation2d rotation = centerPose.getRotation();
    Translation2d rectCenter = centerPose.getTranslation();
    return new Translation2d[] {
      rectCenter.plus(new Translation2d(halfX, halfY).rotateBy(rotation)),
      rectCenter.plus(new Translation2d(-halfX, halfY).rotateBy(rotation)),
      rectCenter.plus(new Translation2d(-halfX, -halfY).rotateBy(rotation)),
      rectCenter.plus(new Translation2d(halfX, -halfY).rotateBy(rotation))
    };
  }

  private static double distancePointToSegment(
      Translation2d point, Translation2d a, Translation2d b) {
    double abX = b.getX() - a.getX();
    double abY = b.getY() - a.getY();
    double apX = point.getX() - a.getX();
    double apY = point.getY() - a.getY();
    double abLenSq = abX * abX + abY * abY;

    if (abLenSq <= 1e-9) {
      return point.getDistance(a);
    }

    double t = (apX * abX + apY * abY) / abLenSq;
    t = Math.max(0.0, Math.min(1.0, t));
    double closestX = a.getX() + t * abX;
    double closestY = a.getY() + t * abY;
    return point.getDistance(new Translation2d(closestX, closestY));
  }
}
