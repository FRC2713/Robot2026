package frc2713.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

class InputShapingTest {
  private static final double EPSILON = 1e-12;

  @Test
  void squareWithSign_scalesSmallValuesAndPreservesSign() {
    Translation2d result = InputShaping.squareWithSign().apply(0.25, -0.5);

    assertEquals(0.0625, result.getX(), EPSILON);
    assertEquals(-0.25, result.getY(), EPSILON);
  }

  @Test
  void mapToCircle_normalizesDiagonalMagnitude() {
    Translation2d result = InputShaping.identity().mapToCircle().apply(1.0, 1.0);

    assertEquals(1.0, result.getNorm(), EPSILON);
    assertEquals(result.getX(), result.getY(), EPSILON);
  }

  @Test
  void deadband_zerosInsideAndRescalesOutside() {
    InputShaping shaping = InputShaping.identity().deadband(0.2);

    Translation2d inside = shaping.apply(0.1, 0.0);
    Translation2d outside = shaping.apply(0.6, 0.0);

    assertEquals(0.0, inside.getNorm(), EPSILON);
    assertEquals(0.5, outside.getX(), EPSILON);
    assertEquals(0.0, outside.getY(), EPSILON);
  }

  @Test
  void chainOrder_changesOutputAsExpected() {
    Translation2d squareThenCircle =
        InputShaping.identity().signedPower(2.0).mapToCircle().apply(1.0, 1.0);
    Translation2d circleThenSquare =
        InputShaping.identity().mapToCircle().signedPower(2.0).apply(1.0, 1.0);

    assertEquals(Math.sqrt(0.5), squareThenCircle.getX(), 1e-9);
    assertEquals(0.5, circleThenSquare.getX(), 1e-12);
    assertNotEquals(squareThenCircle.getX(), circleThenSquare.getX(), 1e-6);
  }
}
