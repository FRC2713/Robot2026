package frc2713.lib.util;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

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
}
