package frc2713.lib.geometry;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class Rectangle2dTest {

  @Nested
  class ContainsRectangleAxisAligned {

    @Test
    void smallerRectInside_returnsTrue() {
      // Outer: (0,0) to (10,10), center (5,5), 10x10
      Rectangle2d outer = new Rectangle2d(new Translation2d(0, 0), new Translation2d(10, 10));
      // Inner: (2,2) to (6,6), fully inside
      Rectangle2d inner = new Rectangle2d(new Translation2d(2, 2), new Translation2d(6, 6));
      assertTrue(outer.contains(inner));
    }

    @Test
    void sameRect_returnsTrue() {
      Rectangle2d rect = new Rectangle2d(new Translation2d(0, 0), new Translation2d(10, 10));
      assertTrue(rect.contains(rect));
    }

    @Test
    void rectExtendingOutside_returnsFalse() {
      Rectangle2d outer = new Rectangle2d(new Translation2d(0, 0), new Translation2d(10, 10));
      Rectangle2d inner = new Rectangle2d(new Translation2d(-1, 2), new Translation2d(6, 6));
      assertFalse(outer.contains(inner));
    }

    @Test
    void rectTouchingEdge_returnsTrue() {
      Rectangle2d outer = new Rectangle2d(new Translation2d(0, 0), new Translation2d(10, 10));
      Rectangle2d inner = new Rectangle2d(new Translation2d(0, 0), new Translation2d(5, 5));
      assertTrue(outer.contains(inner));
    }
  }

  @Nested
  class IntersectsAxisAligned {

    @Test
    void overlappingRects_returnsTrue() {
      Rectangle2d a = new Rectangle2d(new Translation2d(0, 0), new Translation2d(10, 10));
      Rectangle2d b = new Rectangle2d(new Translation2d(5, 5), new Translation2d(15, 15));
      assertTrue(a.intersects(b));
      assertTrue(b.intersects(a));
    }

    @Test
    void overlappingRectsAtBoundary_returnsTrue() {
      Rectangle2d a = new Rectangle2d(new Translation2d(0, 0), new Translation2d(10, 10));
      Rectangle2d b = new Rectangle2d(new Translation2d(9, 0), new Translation2d(19, 10));
      assertTrue(a.intersects(b));
    }

    @Test
    void nonOverlappingRects_returnsFalse() {
      Rectangle2d a = new Rectangle2d(new Translation2d(0, 0), new Translation2d(10, 10));
      Rectangle2d b = new Rectangle2d(new Translation2d(20, 20), new Translation2d(30, 30));
      assertFalse(a.intersects(b));
    }

    @Test
    void oneInsideOther_returnsTrue() {
      Rectangle2d outer = new Rectangle2d(new Translation2d(0, 0), new Translation2d(10, 10));
      Rectangle2d inner = new Rectangle2d(new Translation2d(2, 2), new Translation2d(6, 6));
      assertTrue(outer.intersects(inner));
      assertTrue(inner.intersects(outer));
    }
  }

  @Nested
  class ContainsPoint {

    @Test
    void pointInside_returnsTrue() {
      Rectangle2d rect = new Rectangle2d(new Translation2d(0, 0), new Translation2d(10, 10));
      assertTrue(rect.contains(new Translation2d(5, 5)));
      assertTrue(rect.contains(new Translation2d(1, 1)));
    }

    @Test
    void pointOnBoundary_returnsTrue() {
      Rectangle2d rect = new Rectangle2d(new Translation2d(0, 0), new Translation2d(10, 10));
      assertTrue(rect.contains(new Translation2d(0, 5)));
      assertTrue(rect.contains(new Translation2d(5, 0)));
    }

    @Test
    void pointOutside_returnsFalse() {
      Rectangle2d rect = new Rectangle2d(new Translation2d(0, 0), new Translation2d(10, 10));
      assertFalse(rect.contains(new Translation2d(-1, 5)));
      assertFalse(rect.contains(new Translation2d(15, 15)));
    }
  }

  @Nested
  class RotatedRectangles {

    @Test
    void rotatedRectContainsPoint_returnsTrue() {
      // 4x4 square centered at (5,5), rotated 45 degrees
      Rectangle2d rect = new Rectangle2d(new Pose2d(5, 5, Rotation2d.fromDegrees(45)), 4, 4);
      assertTrue(rect.contains(new Translation2d(5, 5)));
    }

    @Test
    void rotatedRectsIntersecting_returnsTrue() {
      // Two rotated rectangles that overlap (cross shape)
      Rectangle2d a = new Rectangle2d(new Pose2d(5, 5, Rotation2d.kZero), 4, 0.5); // horizontal bar
      Rectangle2d b =
          new Rectangle2d(new Pose2d(5, 5, Rotation2d.fromDegrees(90)), 4, 0.5); // vertical bar
      assertTrue(a.intersects(b));
    }

    @Test
    void rotatedRectContainsSmallerRotatedRect_returnsTrue() {
      Rectangle2d outer = new Rectangle2d(new Pose2d(5, 5, Rotation2d.fromDegrees(30)), 10, 10);
      Rectangle2d inner = new Rectangle2d(new Pose2d(5, 5, Rotation2d.fromDegrees(30)), 2, 2);
      assertTrue(outer.contains(inner));
    }
  }

  @Nested
  class CenterPoseConstructor {

    @Test
    void createsCorrectBounds() {
      Rectangle2d rect = new Rectangle2d(new Pose2d(5, 5, Rotation2d.kZero), 4, 2);
      // Center (5,5), width 4 (x: 3-7), height 2 (y: 4-6)
      assertTrue(rect.contains(new Translation2d(5, 5)));
      assertTrue(rect.contains(new Translation2d(3, 4)));
      assertTrue(rect.contains(new Translation2d(7, 6)));
      assertFalse(rect.contains(new Translation2d(2, 5)));
      assertFalse(rect.contains(new Translation2d(5, 3)));
    }
  }
}
