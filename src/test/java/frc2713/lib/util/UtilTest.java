package frc2713.lib.util;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class UtilTest {

  @Nested
  class EpsilonEqualsDouble {

    @Test
    void equalValues_returnsTrue() {
      assertTrue(Util.epsilonEquals(1.0, 1.0));
      assertTrue(Util.epsilonEquals(0.0, 0.0));
      assertTrue(Util.epsilonEquals(-3.14, -3.14));
    }

    @Test
    void valuesWithinEpsilon_returnsTrue() {
      assertTrue(Util.epsilonEquals(1.0, 1.0 + Util.EPSILON / 2));
      assertTrue(Util.epsilonEquals(1.0, 1.0 - Util.EPSILON / 2));
    }

    @Test
    void valuesOutsideEpsilon_returnsFalse() {
      assertFalse(Util.epsilonEquals(1.0, 1.0 + Util.EPSILON * 2));
      assertFalse(Util.epsilonEquals(1.0, 0.9));
    }

    @Test
    void withCustomEpsilon_respectsEpsilon() {
      assertTrue(Util.epsilonEquals(1.0, 1.0 + 0.01, 0.1));
      assertFalse(Util.epsilonEquals(1.0, 1.0 + 0.01, 0.001));
    }
  }

  @Nested
  class EpsilonEqualsInt {

    @Test
    void equalValues_returnsTrue() {
      assertTrue(Util.epsilonEquals(5, 5));
      assertTrue(Util.epsilonEquals(0, 0));
    }

    @Test
    void valuesWithinEpsilon_returnsTrue() {
      assertTrue(Util.epsilonEquals(10, 11, 1));
      assertTrue(Util.epsilonEquals(10, 9, 1));
    }

    @Test
    void valuesOutsideEpsilon_returnsFalse() {
      assertFalse(Util.epsilonEquals(10, 12, 1));
      assertFalse(Util.epsilonEquals(10, 8, 1));
    }
  }

  @Nested
  class EpsilonEqualsAngle {

    @Test
    void equalAngles_returnsTrue() {
      Angle a = Radians.of(1.0);
      Angle b = Radians.of(1.0);
      Angle eps = Radians.of(Util.EPSILON);
      assertTrue(Util.epsilonEquals(a, b, eps));
    }

    @Test
    void anglesWithinEpsilon_returnsTrue() {
      Angle a = Radians.of(1.0);
      Angle b = Radians.of(1.0 + Util.EPSILON / 2);
      Angle eps = Radians.of(Util.EPSILON);
      assertTrue(Util.epsilonEquals(a, b, eps));
    }

    @Test
    void anglesOutsideEpsilon_returnsFalse() {
      Angle a = Radians.of(1.0);
      Angle b = Radians.of(1.5);
      Angle eps = Radians.of(0.1);
      assertFalse(Util.epsilonEquals(a, b, eps));
    }
  }

  @Nested
  class EpsilonEqualsAngularVelocity {

    @Test
    void equalVelocities_returnsTrue() {
      AngularVelocity a = RadiansPerSecond.of(2.0);
      AngularVelocity b = RadiansPerSecond.of(2.0);
      AngularVelocity eps = RadiansPerSecond.of(Util.EPSILON);
      assertTrue(Util.epsilonEquals(a, b, eps));
    }

    @Test
    void velocitiesOutsideEpsilon_returnsFalse() {
      AngularVelocity a = RadiansPerSecond.of(2.0);
      AngularVelocity b = RadiansPerSecond.of(3.0);
      AngularVelocity eps = RadiansPerSecond.of(0.1);
      assertFalse(Util.epsilonEquals(a, b, eps));
    }
  }

  @Nested
  class FieldToRobotRelative {

    @Test
    void robotFacingZero_returnsSameAngle() {
      Pose2d robotPose = new Pose2d(0, 0, Rotation2d.kZero);
      Angle fieldAngle = Radians.of(Math.PI / 4);
      Angle result = Util.fieldToRobotRelative(fieldAngle, robotPose);
      assertTrue(Util.epsilonEquals(fieldAngle, result, Radians.of(Util.EPSILON)));
    }

    @Test
    void robotRotated90Degrees_convertsCorrectly() {
      Pose2d robotPose = new Pose2d(0, 0, Rotation2d.fromDegrees(90));
      Angle fieldAngle = Radians.of(Math.PI / 2); // 90 deg in field
      Angle result = Util.fieldToRobotRelative(fieldAngle, robotPose);
      // Robot-relative: field 90 - robot 90 = 0
      assertTrue(Util.epsilonEquals(result, Radians.of(0), Radians.of(Util.EPSILON)));
    }

    @Test
    void robotRotated180Degrees_convertsCorrectly() {
      Pose2d robotPose = new Pose2d(0, 0, Rotation2d.kPi);
      Angle fieldAngle = Radians.of(0);
      Angle result = Util.fieldToRobotRelative(fieldAngle, robotPose);
      // Robot-relative: 0 - 180 = -180 or 180 (same rotation)
      double resultRad = result.in(Radians);
      assertTrue(Math.abs(resultRad - Math.PI) < 0.01 || Math.abs(resultRad + Math.PI) < 0.01);
    }
  }
}
