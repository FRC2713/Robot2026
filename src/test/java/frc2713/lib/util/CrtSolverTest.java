package frc2713.lib.util;

import static edu.wpi.first.units.Units.Rotations;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import edu.wpi.first.units.measure.Angle;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class CrtSolverTest {

  private static final double TOLERANCE = 1e-6;

  // Coprime gear pair: 15T motor pinion, 26T driven encoder
  private static final int GEAR_15 = 15;
  private static final int GEAR_26 = 26;

  @Nested
  class ModInverse {

    @Test
    void coprimePair_15and26_returnsCorrectInverse() {
      int inv = CrtSolver.modInverse(15, 26);
      assertEquals(1, (15L * inv) % 26);
    }

    @Test
    void coprimePair_7and11_returnsCorrectInverse() {
      int inv = CrtSolver.modInverse(7, 11);
      assertEquals(1, (7L * inv) % 11);
    }

    @Test
    void coprimePair_3and26_returnsCorrectInverse() {
      int inv = CrtSolver.modInverse(3, 26);
      assertEquals(1, (3L * inv) % 26);
    }

    @Test
    void identity_1andAny_returns1() {
      assertEquals(1, CrtSolver.modInverse(1, 7));
      assertEquals(1, CrtSolver.modInverse(1, 26));
    }

    @Test
    void moduloIs1_returns0() {
      assertEquals(0, CrtSolver.modInverse(5, 1));
    }

    @Test
    void notCoprime_throws() {
      assertThrows(ArithmeticException.class, () -> CrtSolver.modInverse(6, 12));
      assertThrows(ArithmeticException.class, () -> CrtSolver.modInverse(4, 8));
    }
  }

  @Nested
  class CalculateAbsoluteMotorTurns {

    @Test
    void zeroTurns_bothEncodersAtZero() {
      Angle enc1 = Rotations.of(0.0);
      Angle enc2 = Rotations.of(0.0);
      double result = CrtSolver.calculateAbsoluteMotorTurns(enc1, enc2, GEAR_15, GEAR_26);
      assertEquals(0.0, result, TOLERANCE);
    }

    @Test
    void fractionalMotorTurn_noFullRotations() {
      // Motor at 0.5 rotations, encoder2 sees 0.5 * 15/26 = 0.288461... rotations
      double motorTurns = 0.5;
      double enc2Frac = (motorTurns * GEAR_15) / (double) GEAR_26;
      Angle enc1 = Rotations.of(motorTurns);
      Angle enc2 = Rotations.of(enc2Frac);
      double result = CrtSolver.calculateAbsoluteMotorTurns(enc1, enc2, GEAR_15, GEAR_26);
      assertEquals(motorTurns, result, TOLERANCE);
    }

    @Test
    void exactlyOneMotorTurn_enc1WrapsToZero() {
      // After 1 full motor turn, enc1 reads 0.0 again, enc2 reads 15/26
      double motorTurns = 1.0;
      double enc1Frac = motorTurns % 1.0; // 0.0
      double enc2Frac = ((motorTurns * GEAR_15) / (double) GEAR_26) % 1.0;
      Angle enc1 = Rotations.of(enc1Frac);
      Angle enc2 = Rotations.of(enc2Frac);
      double result = CrtSolver.calculateAbsoluteMotorTurns(enc1, enc2, GEAR_15, GEAR_26);
      assertEquals(motorTurns, result, TOLERANCE);
    }

    @Test
    void multipleFullTurns_reconstructsCorrectly() {
      for (int km = 0; km < GEAR_26; km++) {
        double motorPhase = 0.3;
        double totalMotorTurns = km + motorPhase;
        double enc1Frac = totalMotorTurns % 1.0;
        double enc2Frac = ((totalMotorTurns * GEAR_15) / (double) GEAR_26) % 1.0;
        Angle enc1 = Rotations.of(enc1Frac);
        Angle enc2 = Rotations.of(enc2Frac);
        double result = CrtSolver.calculateAbsoluteMotorTurns(enc1, enc2, GEAR_15, GEAR_26);
        assertEquals(
            totalMotorTurns,
            result,
            TOLERANCE,
            "Failed for km=" + km + " (totalMotorTurns=" + totalMotorTurns + ")");
      }
    }

    @Test
    void allIntegerTurns_zeroPhase() {
      for (int km = 0; km < GEAR_26; km++) {
        double enc1Frac = 0.0;
        double enc2Frac = ((km * (double) GEAR_15) / GEAR_26) % 1.0;
        Angle enc1 = Rotations.of(enc1Frac);
        Angle enc2 = Rotations.of(enc2Frac);
        double result = CrtSolver.calculateAbsoluteMotorTurns(enc1, enc2, GEAR_15, GEAR_26);
        assertEquals(km, result, TOLERANCE, "Failed for km=" + km);
      }
    }

    @Test
    void highMotorPhase_nearOneRotation() {
      double motorPhase = 0.99;
      for (int km = 0; km < GEAR_26; km++) {
        double totalMotorTurns = km + motorPhase;
        double enc1Frac = totalMotorTurns % 1.0;
        double enc2Frac = ((totalMotorTurns * GEAR_15) / (double) GEAR_26) % 1.0;
        Angle enc1 = Rotations.of(enc1Frac);
        Angle enc2 = Rotations.of(enc2Frac);
        double result = CrtSolver.calculateAbsoluteMotorTurns(enc1, enc2, GEAR_15, GEAR_26);
        assertEquals(
            totalMotorTurns,
            result,
            TOLERANCE,
            "Failed for km=" + km + " (totalMotorTurns=" + totalMotorTurns + ")");
      }
    }

    @Test
    void differentCoprimePair_7and11() {
      int teeth1 = 7;
      int teeth2 = 11;
      for (int km = 0; km < teeth2; km++) {
        double motorPhase = 0.25;
        double totalMotorTurns = km + motorPhase;
        double enc1Frac = totalMotorTurns % 1.0;
        double enc2Frac = ((totalMotorTurns * teeth1) / (double) teeth2) % 1.0;
        Angle enc1 = Rotations.of(enc1Frac);
        Angle enc2 = Rotations.of(enc2Frac);
        double result = CrtSolver.calculateAbsoluteMotorTurns(enc1, enc2, teeth1, teeth2);
        assertEquals(totalMotorTurns, result, TOLERANCE, "Failed for km=" + km);
      }
    }

    @Test
    void encoderInputsGreaterThanOne_normalizesCorrectly() {
      // Simulate raw encoder reading of 2.3 rotations (should normalize to 0.3)
      double motorPhase = 0.3;
      int km = 5;
      double totalMotorTurns = km + motorPhase;
      double enc2Frac = ((totalMotorTurns * GEAR_15) / (double) GEAR_26) % 1.0;

      Angle enc1 = Rotations.of(motorPhase + 2.0);
      Angle enc2 = Rotations.of(enc2Frac + 3.0);
      double result = CrtSolver.calculateAbsoluteMotorTurns(enc1, enc2, GEAR_15, GEAR_26);
      assertEquals(totalMotorTurns, result, TOLERANCE);
    }

    @Test
    void lastValidTurn_maxRange() {
      // km = 25 (last valid turn in range [0, 26))
      int km = GEAR_26 - 1;
      double motorPhase = 0.5;
      double totalMotorTurns = km + motorPhase;
      double enc1Frac = totalMotorTurns % 1.0;
      double enc2Frac = ((totalMotorTurns * GEAR_15) / (double) GEAR_26) % 1.0;
      Angle enc1 = Rotations.of(enc1Frac);
      Angle enc2 = Rotations.of(enc2Frac);
      double result = CrtSolver.calculateAbsoluteMotorTurns(enc1, enc2, GEAR_15, GEAR_26);
      assertEquals(totalMotorTurns, result, TOLERANCE);
    }
  }
}
