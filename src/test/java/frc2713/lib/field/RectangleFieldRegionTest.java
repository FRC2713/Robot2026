package frc2713.lib.field;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class RectangleFieldRegionTest {

  @Nested
  class ContainsPoint {

    @Test
    void pointInside_returnsTrue() {
      RectangleFieldRegion region =
          new RectangleFieldRegion(new Translation2d(0, 0), new Translation2d(10, 10));
      assertTrue(region.contains(new Translation2d(5, 5)));
    }

    @Test
    void pointOutside_returnsFalse() {
      RectangleFieldRegion region =
          new RectangleFieldRegion(new Translation2d(0, 0), new Translation2d(10, 10));
      assertFalse(region.contains(new Translation2d(15, 15)));
    }

    @Test
    void pointOnBoundary_returnsTrue() {
      RectangleFieldRegion region =
          new RectangleFieldRegion(new Translation2d(0, 0), new Translation2d(10, 10));
      assertTrue(region.contains(new Translation2d(0, 5)));
    }
  }

  @Nested
  class ContainsRectangle {

    @Test
    void smallerRectInside_returnsTrue() {
      RectangleFieldRegion region =
          new RectangleFieldRegion(new Translation2d(0, 0), new Translation2d(10, 10));
      var inner =
          new edu.wpi.first.math.geometry.Rectangle2d(
              new Translation2d(2, 2), new Translation2d(6, 6));
      assertTrue(region.contains(inner));
    }

    @Test
    void rectExtendingOutside_returnsFalse() {
      RectangleFieldRegion region =
          new RectangleFieldRegion(new Translation2d(0, 0), new Translation2d(10, 10));
      var inner =
          new edu.wpi.first.math.geometry.Rectangle2d(
              new Translation2d(-1, 2), new Translation2d(6, 6));
      assertFalse(region.contains(inner));
    }
  }

  @Nested
  class Intersects {

    @Test
    void overlappingRects_returnsTrue() {
      RectangleFieldRegion region =
          new RectangleFieldRegion(new Translation2d(0, 0), new Translation2d(10, 10));
      var other =
          new edu.wpi.first.math.geometry.Rectangle2d(
              new Translation2d(5, 5), new Translation2d(15, 15));
      assertTrue(region.intersects(other));
    }

    @Test
    void nonOverlappingRects_returnsFalse() {
      RectangleFieldRegion region =
          new RectangleFieldRegion(new Translation2d(0, 0), new Translation2d(10, 10));
      var other =
          new edu.wpi.first.math.geometry.Rectangle2d(
              new Translation2d(20, 20), new Translation2d(30, 30));
      assertFalse(region.intersects(other));
    }
  }

  @Nested
  class CenterPoseConstructor {

    @Test
    void rotatedRegionContainsPoint() {
      RectangleFieldRegion region =
          new RectangleFieldRegion(new Pose2d(5, 5, Rotation2d.fromDegrees(45)), 4, 4);
      assertTrue(region.contains(new Translation2d(5, 5)));
    }

    @Test
    void rotatedRegionIntersectsRect() {
      RectangleFieldRegion region =
          new RectangleFieldRegion(new Pose2d(5, 5, Rotation2d.kZero), 4, 4);
      var other =
          new edu.wpi.first.math.geometry.Rectangle2d(
              new Translation2d(4, 4), new Translation2d(8, 8));
      assertTrue(region.intersects(other));
    }
  }
}
