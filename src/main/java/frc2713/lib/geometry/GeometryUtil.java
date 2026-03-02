package frc2713.lib.geometry;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;

public class GeometryUtil {
  public static Angle angleModulus(Angle angle, Angle minOutput, Angle maxOutput) {
    return Degrees.of(
        MathUtil.inputModulus(angle.in(Degrees), minOutput.in(Degrees), maxOutput.in(Degrees)));
  }
}
