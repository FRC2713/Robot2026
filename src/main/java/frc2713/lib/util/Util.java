package frc2713.lib.util;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc2713.robot.Robot;

public class Util {
  public static final double EPSILON = 1e-12;

  public Util() {}

  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  public static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, EPSILON);
  }

  public static boolean epsilonEquals(int a, int b, int epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  public static boolean epsilonEquals(Angle a, Angle b, Angle epsilon) {
    return (a.minus(epsilon).lte(b)) && (a.plus(epsilon).gte(b));
  }

  public static boolean epsilonEquals(
      AngularVelocity a, AngularVelocity b, AngularVelocity epsilon) {
    return (a.minus(epsilon).lte(b)) && (a.plus(epsilon).gte(b));
  }

  public static Angle fieldToRobotRelative(Angle robotRelative, Pose2d robotPose) {
    Rotation2d converted = new Rotation2d(robotRelative.in(Radians)).minus(robotPose.getRotation());
    return converted.getMeasure();
  }

  public static <T> T modeDependentValue(T real, T sim) {
    return Robot.isReal() ? real : sim;
  }

  public static <T> T modeDependentValue(T real) {
    return modeDependentValue(real, real);
  }

  public static Angle clamp(Angle value, Angle min, Angle max) {
    if (value.lt(min)) {
      return min;
    } else if (value.gt(max)) {
      return max;
    } else {
      return value;
    }
  }

  /**
   * Maps an x/y input pair onto the unit circle.
   *
   * <p>If the input magnitude is less than or equal to 1, values are returned unchanged. If the
   * magnitude is greater than 1 (for example x=1, y=1), the vector is normalized so the resulting
   * magnitude is exactly 1 while preserving direction.
   */
  public static Translation2d mapToUnitCircle(double x, double y) {
    double magnitude = Math.hypot(x, y);
    if (magnitude <= 1.0 || magnitude == 0.0) {
      return new Translation2d(x, y);
    }
    return new Translation2d(x / magnitude, y / magnitude);
  }

  /** Applies a signed power curve to an input while preserving sign. */
  public static double signedPower(double value, double exponent) {
    if (exponent <= 0.0) {
      throw new IllegalArgumentException("Exponent must be greater than zero.");
    }
    return Math.copySign(Math.pow(Math.abs(value), exponent), value);
  }

  /** Squares an input while preserving sign (e.g. -0.5 -> -0.25). */
  public static double squareWithSign(double value) {
    return signedPower(value, 2.0);
  }

  /** Squares joystick x/y values while preserving sign on each axis. */
  public static Translation2d squareWithSign(double x, double y) {
    return new Translation2d(squareWithSign(x), squareWithSign(y));
  }
}
