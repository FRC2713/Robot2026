package frc2713.lib.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Extended Rectangle2d with rectangle-rectangle geometry support. Extends WPILib's Rectangle2d with
 * {@link #contains(Rectangle2d)} for containment checks and {@link #intersects} for overlap checks.
 */
public class Rectangle2d extends edu.wpi.first.math.geometry.Rectangle2d {

  public Rectangle2d(Translation2d corner1, Translation2d corner2) {
    super(corner1, corner2);
  }

  public Rectangle2d(Pose2d center, double xWidth, double yWidth) {
    super(center, xWidth, yWidth);
  }

  /**
   * Returns true if the given rectangle is entirely contained within this rectangle (all corners and
   * edges inside or on the boundary).
   */
  public boolean contains(edu.wpi.first.math.geometry.Rectangle2d rectangle) {
    // Fast path: both axis-aligned
    if (getRotation().equals(Rotation2d.kZero)
        && rectangle.getRotation().equals(Rotation2d.kZero)) {
      return aabbContains(
          getCenter().getX(),
          getCenter().getY(),
          getXWidth(),
          getYWidth(),
          rectangle.getCenter().getX(),
          rectangle.getCenter().getY(),
          rectangle.getXWidth(),
          rectangle.getYWidth());
    }

    // General case: all four corners of the other rectangle must be inside this one
    Translation2d[] otherCorners = getCorners(rectangle);
    for (Translation2d corner : otherCorners) {
      if (!contains(corner)) {
        return false;
      }
    }
    return true;
  }

  /**
   * Returns true if this rectangle intersects the given rectangle (including edges and corners).
   */
  public boolean intersects(edu.wpi.first.math.geometry.Rectangle2d rectangle) {
    // Fast path: both axis-aligned (no allocations, 4 comparisons)
    if (getRotation().equals(Rotation2d.kZero)
        && rectangle.getRotation().equals(Rotation2d.kZero)) {
      return aabbOverlaps(
          getCenter().getX(),
          getCenter().getY(),
          getXWidth(),
          getYWidth(),
          rectangle.getCenter().getX(),
          rectangle.getCenter().getY(),
          rectangle.getXWidth(),
          rectangle.getYWidth());
    }

    Translation2d[] myCorners = getCorners(this);
    Translation2d[] otherCorners = getCorners(rectangle);

    // AABB early rejection - if bounding boxes don't overlap, can't intersect
    double myMinX = minX(myCorners);
    double myMaxX = maxX(myCorners);
    double myMinY = minY(myCorners);
    double myMaxY = maxY(myCorners);
    double otherMinX = minX(otherCorners);
    double otherMaxX = maxX(otherCorners);
    double otherMinY = minY(otherCorners);
    double otherMaxY = maxY(otherCorners);
    if (myMaxX <= otherMinX || myMinX >= otherMaxX || myMaxY <= otherMinY || myMinY >= otherMaxY) {
      return false;
    }

    // Check if any corner of the other rectangle is inside this one
    for (Translation2d corner : otherCorners) {
      if (contains(corner)) {
        return true;
      }
    }

    // Check if any corner of this rectangle is inside the other one
    for (Translation2d corner : myCorners) {
      if (rectangle.contains(corner)) {
        return true;
      }
    }

    // Check edge intersections (handles crossing without corners inside)
    for (int i = 0; i < 4; i++) {
      Translation2d a1 = myCorners[i];
      Translation2d a2 = myCorners[(i + 1) % 4];
      for (int j = 0; j < 4; j++) {
        Translation2d b1 = otherCorners[j];
        Translation2d b2 = otherCorners[(j + 1) % 4];
        if (segmentsIntersect(a1, a2, b1, b2)) {
          return true;
        }
      }
    }

    return false;
  }

  private static boolean aabbOverlaps(
      double cx1, double cy1, double w1, double h1, double cx2, double cy2, double w2, double h2) {
    double halfW1 = w1 / 2.0;
    double halfH1 = h1 / 2.0;
    double halfW2 = w2 / 2.0;
    double halfH2 = h2 / 2.0;
    return cx1 - halfW1 < cx2 + halfW2
        && cx1 + halfW1 > cx2 - halfW2
        && cy1 - halfH1 < cy2 + halfH2
        && cy1 + halfH1 > cy2 - halfH2;
  }

  private static boolean aabbContains(
      double cx1, double cy1, double w1, double h1, double cx2, double cy2, double w2, double h2) {
    double halfW1 = w1 / 2.0;
    double halfH1 = h1 / 2.0;
    double halfW2 = w2 / 2.0;
    double halfH2 = h2 / 2.0;
    return cx1 - halfW1 <= cx2 - halfW2
        && cx1 + halfW1 >= cx2 + halfW2
        && cy1 - halfH1 <= cy2 - halfH2
        && cy1 + halfH1 >= cy2 + halfH2;
  }

  private static double minX(Translation2d[] corners) {
    double m = corners[0].getX();
    for (int i = 1; i < 4; i++) {
      m = Math.min(m, corners[i].getX());
    }
    return m;
  }

  private static double maxX(Translation2d[] corners) {
    double m = corners[0].getX();
    for (int i = 1; i < 4; i++) {
      m = Math.max(m, corners[i].getX());
    }
    return m;
  }

  private static double minY(Translation2d[] corners) {
    double m = corners[0].getY();
    for (int i = 1; i < 4; i++) {
      m = Math.min(m, corners[i].getY());
    }
    return m;
  }

  private static double maxY(Translation2d[] corners) {
    double m = corners[0].getY();
    for (int i = 1; i < 4; i++) {
      m = Math.max(m, corners[i].getY());
    }
    return m;
  }

  private static Translation2d[] getCorners(edu.wpi.first.math.geometry.Rectangle2d rect) {
    double hx = rect.getXWidth() / 2.0;
    double hy = rect.getYWidth() / 2.0;
    var center = rect.getCenter();
    var rot = center.getRotation();
    var t = center.getTranslation();
    return new Translation2d[] {
      t.plus(new Translation2d(hx, hy).rotateBy(rot)),
      t.plus(new Translation2d(-hx, hy).rotateBy(rot)),
      t.plus(new Translation2d(-hx, -hy).rotateBy(rot)),
      t.plus(new Translation2d(hx, -hy).rotateBy(rot))
    };
  }

  private static boolean segmentsIntersect(
      Translation2d a1, Translation2d a2, Translation2d b1, Translation2d b2) {
    double ax = a2.getX() - a1.getX();
    double ay = a2.getY() - a1.getY();
    double d1 =
        cross(ax, ay, b1.getX() - a1.getX(), b1.getY() - a1.getY())
            * cross(ax, ay, b2.getX() - a1.getX(), b2.getY() - a1.getY());
    double bx = b2.getX() - b1.getX();
    double by = b2.getY() - b1.getY();
    double d2 =
        cross(bx, by, a1.getX() - b1.getX(), a1.getY() - b1.getY())
            * cross(bx, by, a2.getX() - b1.getX(), a2.getY() - b1.getY());
    return d1 <= 0 && d2 <= 0;
  }

  private static double cross(double ax, double ay, double bx, double by) {
    return ax * by - ay * bx;
  }
}
