package frc2713.lib.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Extended Rectangle2d with rectangle-rectangle geometry support.
 *
 * <p>Extends WPILib's {@link edu.wpi.first.math.geometry.Rectangle2d} with {@link
 * #contains(edu.wpi.first.math.geometry.Rectangle2d)} for containment checks and {@link
 * #intersects} for overlap checks. Supports both axis-aligned and rotated rectangles. All
 * dimensions are in meters.
 */
public class Rectangle2d extends edu.wpi.first.math.geometry.Rectangle2d {

  /**
   * Constructs an axis-aligned rectangle from two diagonally opposite corners.
   *
   * @param corner1 First corner of the rectangle
   * @param corner2 Second corner (diagonal from corner1)
   */
  public Rectangle2d(Translation2d corner1, Translation2d corner2) {
    super(corner1, corner2);
  }

  /**
   * Constructs a rectangle from its center pose and dimensions.
   *
   * @param center Center position and rotation of the rectangle
   * @param xWidth Width along the local x-axis (before rotation)
   * @param yWidth Height along the local y-axis (before rotation)
   */
  public Rectangle2d(Pose2d center, double xWidth, double yWidth) {
    super(center, xWidth, yWidth);
  }

  /**
   * Returns true if the given rectangle is entirely contained within this rectangle.
   *
   * <p>All four corners and edges of the other rectangle must lie inside or on the boundary of this
   * rectangle.
   *
   * @param rectangle The rectangle to check for containment
   * @return true if the rectangle is fully inside this one
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
   * Returns true if this rectangle intersects the given rectangle.
   *
   * <p>Intersection includes any overlap: corners touching, edges crossing (e.g. "+" shape), or one
   * rectangle fully inside the other.
   *
   * @param rectangle The rectangle to check for intersection
   * @return true if the rectangles overlap
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

  /**
   * Checks if two axis-aligned rectangles overlap. Parameters are center (cx, cy) and dimensions
   * (w, h) for each rectangle.
   */
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

  /**
   * Checks if the first axis-aligned rectangle fully contains the second. Parameters are center
   * (cx, cy) and dimensions (w, h) for each rectangle.
   */
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

  /** Returns the minimum X coordinate among the four corners. */
  private static double minX(Translation2d[] corners) {
    double m = corners[0].getX();
    for (int i = 1; i < 4; i++) {
      m = Math.min(m, corners[i].getX());
    }
    return m;
  }

  /** Returns the maximum X coordinate among the four corners. */
  private static double maxX(Translation2d[] corners) {
    double m = corners[0].getX();
    for (int i = 1; i < 4; i++) {
      m = Math.max(m, corners[i].getX());
    }
    return m;
  }

  /** Returns the minimum Y coordinate among the four corners. */
  private static double minY(Translation2d[] corners) {
    double m = corners[0].getY();
    for (int i = 1; i < 4; i++) {
      m = Math.min(m, corners[i].getY());
    }
    return m;
  }

  /** Returns the maximum Y coordinate among the four corners. */
  private static double maxY(Translation2d[] corners) {
    double m = corners[0].getY();
    for (int i = 1; i < 4; i++) {
      m = Math.max(m, corners[i].getY());
    }
    return m;
  }

  /**
   * Returns the four corners of the rectangle in global coordinates (order: +x+y, -x+y, -x-y,
   * +x-y).
   */
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

  /**
   * Returns true if line segment (a1, a2) intersects segment (b1, b2). Uses cross-product
   * orientation test; treats collinear overlapping segments as intersecting.
   */
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

  /** 2D cross product: ax*by - ay*bx. Sign indicates relative orientation. */
  private static double cross(double ax, double ay, double bx, double by) {
    return ax * by - ay * bx;
  }
}
