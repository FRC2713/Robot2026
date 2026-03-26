package frc2713.lib.geometry;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;

public class GeometryUtil {
  public static Angle angleModulus(Angle angle, Angle minOutput, Angle maxOutput) {
    return Degrees.of(
        MathUtil.inputModulus(angle.in(Degrees), minOutput.in(Degrees), maxOutput.in(Degrees)));
  }

  public static class Constants {
    public static class Translation_3d {
      public static Translation3d ZERO = new Translation3d();
    }

    public static class Transform_3d {

      public static Transform3d ZERO = new Transform3d();
    }

    public static class Rotation_3d {
      public static Rotation3d PI_Z = new Rotation3d(0, 0, Math.PI);
      public static Rotation3d PI_Y = new Rotation3d(0, Math.PI, 0);
      public static Rotation3d PI_X = new Rotation3d(Math.PI, 0, 0);
    }
  }
}
