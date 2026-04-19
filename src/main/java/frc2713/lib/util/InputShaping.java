package frc2713.lib.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Objects;
import java.util.function.UnaryOperator;

/** Immutable, composable shaping pipeline for XY controller inputs. */
public final class InputShaping {
  private final UnaryOperator<Translation2d> pipeline;

  private InputShaping(UnaryOperator<Translation2d> pipeline) {
    this.pipeline = pipeline;
  }

  public static InputShaping identity() {
    return new InputShaping(value -> value);
  }

  /** Starts a shaping pipeline with signed square on each axis. */
  public static InputShaping squareWithSign() {
    return identity().signedPower(2.0);
  }

  /** Adds a signed power curve on each axis while preserving sign. */
  public InputShaping signedPower(double exponent) {
    if (exponent <= 0.0) {
      throw new IllegalArgumentException("Exponent must be greater than zero.");
    }
    return andThen(
        value ->
            new Translation2d(
                Math.copySign(Math.pow(Math.abs(value.getX()), exponent), value.getX()),
                Math.copySign(Math.pow(Math.abs(value.getY()), exponent), value.getY())));
  }

  /** Adds unit-circle mapping so output magnitude never exceeds 1. */
  public InputShaping mapToCircle() {
    return andThen(
        value -> {
          double magnitude = value.getNorm();
          if (magnitude <= 1.0 || magnitude == 0.0) {
            return value;
          }
          return value.div(magnitude);
        });
  }

  /** Adds radial deadband while preserving direction. */
  public InputShaping deadband(double deadband) {
    if (deadband < 0.0 || deadband >= 1.0) {
      throw new IllegalArgumentException("Deadband must be in the range [0, 1).");
    }
    return andThen(
        value -> {
          double magnitude = value.getNorm();
          double adjustedMagnitude = MathUtil.applyDeadband(magnitude, deadband);
          if (magnitude == 0.0 || adjustedMagnitude == 0.0) {
            return Translation2d.kZero;
          }
          return value.times(adjustedMagnitude / magnitude);
        });
  }

  public Translation2d apply(double x, double y) {
    return apply(new Translation2d(x, y));
  }

  public Translation2d apply(Translation2d value) {
    return pipeline.apply(Objects.requireNonNull(value));
  }

  private InputShaping andThen(UnaryOperator<Translation2d> next) {
    return new InputShaping(value -> next.apply(pipeline.apply(value)));
  }
}
