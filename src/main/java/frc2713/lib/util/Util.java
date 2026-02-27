package frc2713.lib.util;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
}
