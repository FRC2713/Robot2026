package frc2713.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class GamePieceConstants {
  public static class Fuel {
    public static final Mass mass = Pounds.of(0.5);
    public static final Distance diameter = Meters.of(0.15);
    public static final Distance radius = diameter.div(2);
    public static final double volumeInchesCubed =
        4.0 / 3.0 * Math.PI * Math.pow(radius.in(Inches), 3);

    public static final double dragCoeff = 0.47;
    public static final double liftCoeff = 0.2;
  }
}
