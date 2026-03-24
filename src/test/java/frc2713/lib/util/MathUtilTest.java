package frc2713.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class MathUtilTest {

  @Nested
  class PowInt {

    @Test
    void basicPositiveExponents() {
      assertEquals(1, MathUtil.powInt(2, 0));
      assertEquals(2, MathUtil.powInt(2, 1));
      assertEquals(4, MathUtil.powInt(2, 2));
      assertEquals(8, MathUtil.powInt(2, 3));
      assertEquals(16, MathUtil.powInt(2, 4));
      assertEquals(32, MathUtil.powInt(2, 5));
    }

    @Test
    void baseZero() {
      assertEquals(1, MathUtil.powInt(0, 0));
      assertEquals(0, MathUtil.powInt(0, 1));
      assertEquals(0, MathUtil.powInt(0, 5));
    }

    @Test
    void baseOne() {
      assertEquals(1, MathUtil.powInt(1, 0));
      assertEquals(1, MathUtil.powInt(1, 10));
      assertEquals(1, MathUtil.powInt(1, 100));
    }

    @Test
    void negativeBase() {
      assertEquals(1, MathUtil.powInt(-2, 0));
      assertEquals(-2, MathUtil.powInt(-2, 1));
      assertEquals(4, MathUtil.powInt(-2, 2));
      assertEquals(-8, MathUtil.powInt(-2, 3));
      assertEquals(16, MathUtil.powInt(-2, 4));
      assertEquals(-32, MathUtil.powInt(-2, 5));
    }

    @Test
    void negativeExponent_returnsZero() {
      assertEquals(0, MathUtil.powInt(2, -1));
      assertEquals(0, MathUtil.powInt(5, -3));
      assertEquals(0, MathUtil.powInt(-2, -2));
    }

    @Test
    void largerBases() {
      assertEquals(1000, MathUtil.powInt(10, 3));
      assertEquals(125, MathUtil.powInt(5, 3));
      assertEquals(81, MathUtil.powInt(3, 4));
    }

    @Test
    void largeExponents() {
      assertEquals(1024, MathUtil.powInt(2, 10));
      assertEquals(59049, MathUtil.powInt(3, 10));
    }
  }

  @Nested
  class PowDouble {

    private static final double EPSILON = 1e-10;

    @Test
    void basicPositiveExponents() {
      assertEquals(1.0, MathUtil.powDouble(2.0, 0), EPSILON);
      assertEquals(2.0, MathUtil.powDouble(2.0, 1), EPSILON);
      assertEquals(4.0, MathUtil.powDouble(2.0, 2), EPSILON);
      assertEquals(8.0, MathUtil.powDouble(2.0, 3), EPSILON);
      assertEquals(16.0, MathUtil.powDouble(2.0, 4), EPSILON);
    }

    @Test
    void negativeExponents() {
      assertEquals(0.5, MathUtil.powDouble(2.0, -1), EPSILON);
      assertEquals(0.25, MathUtil.powDouble(2.0, -2), EPSILON);
      assertEquals(0.125, MathUtil.powDouble(2.0, -3), EPSILON);
      assertEquals(0.1, MathUtil.powDouble(10.0, -1), EPSILON);
      assertEquals(0.01, MathUtil.powDouble(10.0, -2), EPSILON);
    }

    @Test
    void baseZero() {
      assertEquals(1.0, MathUtil.powDouble(0.0, 0), EPSILON);
      assertEquals(0.0, MathUtil.powDouble(0.0, 1), EPSILON);
      assertEquals(0.0, MathUtil.powDouble(0.0, 5), EPSILON);
    }

    @Test
    void baseOne() {
      assertEquals(1.0, MathUtil.powDouble(1.0, 0), EPSILON);
      assertEquals(1.0, MathUtil.powDouble(1.0, 10), EPSILON);
      assertEquals(1.0, MathUtil.powDouble(1.0, -5), EPSILON);
    }

    @Test
    void negativeBase() {
      assertEquals(1.0, MathUtil.powDouble(-2.0, 0), EPSILON);
      assertEquals(-2.0, MathUtil.powDouble(-2.0, 1), EPSILON);
      assertEquals(4.0, MathUtil.powDouble(-2.0, 2), EPSILON);
      assertEquals(-8.0, MathUtil.powDouble(-2.0, 3), EPSILON);
      assertEquals(16.0, MathUtil.powDouble(-2.0, 4), EPSILON);
    }

    @Test
    void negativeBaseWithNegativeExponent() {
      assertEquals(-0.5, MathUtil.powDouble(-2.0, -1), EPSILON);
      assertEquals(0.25, MathUtil.powDouble(-2.0, -2), EPSILON);
      assertEquals(-0.125, MathUtil.powDouble(-2.0, -3), EPSILON);
    }

    @Test
    void fractionalBases() {
      assertEquals(0.25, MathUtil.powDouble(0.5, 2), EPSILON);
      assertEquals(0.125, MathUtil.powDouble(0.5, 3), EPSILON);
      assertEquals(2.0, MathUtil.powDouble(0.5, -1), EPSILON);
      assertEquals(4.0, MathUtil.powDouble(0.5, -2), EPSILON);
    }

    @Test
    void largeExponents() {
      assertEquals(1024.0, MathUtil.powDouble(2.0, 10), EPSILON);
      assertEquals(Math.pow(3.0, 20), MathUtil.powDouble(3.0, 20), EPSILON);
    }
  }

  @Nested
  class SquareDouble {

    private static final double EPSILON = 1e-10;

    @Test
    void positiveValues() {
      assertEquals(0.0, MathUtil.sq(0.0), EPSILON);
      assertEquals(1.0, MathUtil.sq(1.0), EPSILON);
      assertEquals(4.0, MathUtil.sq(2.0), EPSILON);
      assertEquals(9.0, MathUtil.sq(3.0), EPSILON);
      assertEquals(100.0, MathUtil.sq(10.0), EPSILON);
    }

    @Test
    void negativeValues() {
      assertEquals(1.0, MathUtil.sq(-1.0), EPSILON);
      assertEquals(4.0, MathUtil.sq(-2.0), EPSILON);
      assertEquals(9.0, MathUtil.sq(-3.0), EPSILON);
      assertEquals(100.0, MathUtil.sq(-10.0), EPSILON);
    }

    @Test
    void fractionalValues() {
      assertEquals(0.25, MathUtil.sq(0.5), EPSILON);
      assertEquals(0.01, MathUtil.sq(0.1), EPSILON);
      assertEquals(2.25, MathUtil.sq(1.5), EPSILON);
    }
  }

  @Nested
  class SquareFloat {

    private static final float EPSILON = 1e-6f;

    @Test
    void positiveValues() {
      assertEquals(0.0f, MathUtil.sq(0.0f), EPSILON);
      assertEquals(1.0f, MathUtil.sq(1.0f), EPSILON);
      assertEquals(4.0f, MathUtil.sq(2.0f), EPSILON);
      assertEquals(9.0f, MathUtil.sq(3.0f), EPSILON);
      assertEquals(100.0f, MathUtil.sq(10.0f), EPSILON);
    }

    @Test
    void negativeValues() {
      assertEquals(1.0f, MathUtil.sq(-1.0f), EPSILON);
      assertEquals(4.0f, MathUtil.sq(-2.0f), EPSILON);
      assertEquals(9.0f, MathUtil.sq(-3.0f), EPSILON);
    }

    @Test
    void fractionalValues() {
      assertEquals(0.25f, MathUtil.sq(0.5f), EPSILON);
      assertEquals(0.01f, MathUtil.sq(0.1f), EPSILON);
    }
  }

  @Nested
  class SquareLong {

    @Test
    void positiveValues() {
      assertEquals(0L, MathUtil.sq(0L));
      assertEquals(1L, MathUtil.sq(1L));
      assertEquals(4L, MathUtil.sq(2L));
      assertEquals(9L, MathUtil.sq(3L));
      assertEquals(100L, MathUtil.sq(10L));
      assertEquals(10000L, MathUtil.sq(100L));
    }

    @Test
    void negativeValues() {
      assertEquals(1L, MathUtil.sq(-1L));
      assertEquals(4L, MathUtil.sq(-2L));
      assertEquals(9L, MathUtil.sq(-3L));
      assertEquals(100L, MathUtil.sq(-10L));
    }

    @Test
    void largeValues() {
      assertEquals(1000000L, MathUtil.sq(1000L));
      assertEquals(10000000000L, MathUtil.sq(100000L));
    }
  }

  @Nested
  class SquareInt {

    @Test
    void positiveValues() {
      assertEquals(0, MathUtil.sq(0));
      assertEquals(1, MathUtil.sq(1));
      assertEquals(4, MathUtil.sq(2));
      assertEquals(9, MathUtil.sq(3));
      assertEquals(100, MathUtil.sq(10));
      assertEquals(10000, MathUtil.sq(100));
    }

    @Test
    void negativeValues() {
      assertEquals(1, MathUtil.sq(-1));
      assertEquals(4, MathUtil.sq(-2));
      assertEquals(9, MathUtil.sq(-3));
      assertEquals(100, MathUtil.sq(-10));
    }

    @Test
    void largeValues() {
      assertEquals(1000000, MathUtil.sq(1000));
      assertEquals(40000, MathUtil.sq(200));
    }
  }

  @Nested
  class CachedPowTest {

    private static final double EPSILON = 1e-10;

    @Test
    void firstCall_computesValue() {
      CachedPow cache = new CachedPow(2.0);
      double result = cache.get(3.0);
      assertEquals(Math.pow(3.0, 2.0), result, EPSILON);
    }

    @Test
    void sameInput_returnsCachedValue() {
      CachedPow cache = new CachedPow(2.0);
      double first = cache.get(3.0);
      double second = cache.get(3.0);
      assertEquals(first, second, EPSILON);
      assertEquals(9.0, second, EPSILON);
    }

    @Test
    void inputWithinThreshold_returnsCachedValue() {
      CachedPow cache = new CachedPow(2.0, 0.1);
      double first = cache.get(5.0);
      double second = cache.get(5.05); // within 0.1 threshold
      assertEquals(first, second, EPSILON);
      assertEquals(25.0, second, EPSILON); // still returns 5^2, not 5.05^2
    }

    @Test
    void inputBeyondThreshold_recomputesValue() {
      CachedPow cache = new CachedPow(2.0, 0.1);
      double first = cache.get(5.0);
      double second = cache.get(5.2); // beyond 0.1 threshold
      assertEquals(25.0, first, EPSILON);
      assertEquals(Math.pow(5.2, 2.0), second, EPSILON);
      assertEquals(27.04, second, EPSILON);
    }

    @Test
    void defaultThreshold_is0point001() {
      CachedPow cache = new CachedPow(2.0);
      double first = cache.get(10.0);
      double withinThreshold = cache.get(10.0005); // within 0.001
      double beyondThreshold = cache.get(10.002); // beyond 0.001

      assertEquals(100.0, first, EPSILON);
      assertEquals(100.0, withinThreshold, EPSILON); // cached
      assertEquals(Math.pow(10.002, 2.0), beyondThreshold, EPSILON); // recomputed
    }

    @Test
    void reset_forcesRecomputation() {
      CachedPow cache = new CachedPow(2.0);
      cache.get(5.0);
      cache.reset();
      double result = cache.get(5.0);
      assertEquals(25.0, result, EPSILON);
    }

    @Test
    void fractionalExponent() {
      CachedPow cache = new CachedPow(0.5); // square root
      double result = cache.get(16.0);
      assertEquals(4.0, result, EPSILON);
    }

    @Test
    void negativeExponent() {
      CachedPow cache = new CachedPow(-1.0); // reciprocal
      double result = cache.get(4.0);
      assertEquals(0.25, result, EPSILON);
    }

    @Test
    void sequentialCalls_respectThreshold() {
      CachedPow cache = new CachedPow(3.0, 0.5);

      assertEquals(8.0, cache.get(2.0), EPSILON); // 2^3 = 8
      assertEquals(8.0, cache.get(2.2), EPSILON); // cached (within 0.5)
      assertEquals(8.0, cache.get(2.4), EPSILON); // cached (within 0.5 of last input 2.0)
      assertEquals(Math.pow(2.6, 3.0), cache.get(2.6), EPSILON); // recomputed (beyond 0.5)
    }

    @Test
    void negativeInputs() {
      CachedPow cache = new CachedPow(2.0);
      double result = cache.get(-3.0);
      assertEquals(9.0, result, EPSILON);
    }

    @Test
    void zeroInput() {
      CachedPow cache = new CachedPow(2.0);
      double result = cache.get(0.0);
      assertEquals(0.0, result, EPSILON);
    }

    @Test
    void zeroExponent() {
      CachedPow cache = new CachedPow(0.0);
      double result = cache.get(5.0);
      assertEquals(1.0, result, EPSILON); // anything^0 = 1
    }

    @Test
    void customThreshold_largeValue() {
      CachedPow cache = new CachedPow(2.0, 10.0);
      double first = cache.get(100.0);
      double second = cache.get(105.0); // within threshold of 10
      assertEquals(10000.0, first, EPSILON);
      assertEquals(10000.0, second, EPSILON); // cached
    }

    @Test
    void customThreshold_smallValue() {
      CachedPow cache = new CachedPow(2.0, 0.0001);
      double first = cache.get(5.0);
      double second = cache.get(5.00005); // within 0.0001
      double third = cache.get(5.00015); // beyond 0.0001
      assertEquals(25.0, first, EPSILON);
      assertEquals(25.0, second, EPSILON); // cached
      assertEquals(Math.pow(5.00015, 2.0), third, EPSILON); // recomputed
    }
  }
}
