package frc2713.lib.dynamics;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.derive;

import edu.wpi.first.units.MomentOfInertiaUnit;

/**
 * Extended moment of inertia units not included in {@link edu.wpi.first.units.Units}.
 *
 * <p>These units integrate with the WPILib units system. Use {@link #PoundSquareInches} with {@code
 * .of()} and {@code .in(KilogramSquareMeters)} for conversions.
 *
 * <p>Conversion: 1 lb·in² = 0.45359237 kg × (0.0254 m)² ≈ 2.926397e-4 kg·m²
 */
public final class MoiUnits {

  private MoiUnits() {}

  /**
   * Moment of inertia in lb²·in (pound squared inches). Common in imperial mechanical specs.
   *
   * <p>Example usage:
   *
   * <pre>{@code
   * MomentOfInertia moi = PoundSquareInches.of(100);
   * double kg2M = moi.in(KilogramSquareMeters);
   * }</pre>
   */
  public static final MomentOfInertiaUnit PoundSquareInches =
      derive(KilogramSquareMeters)
          .aggregate(0.45359237 * 0.0254 * 0.0254)
          .named("PoundSquareInch")
          .symbol("lb²·in")
          .make();

  /** Alias for {@link #PoundSquareInches}. */
  public static final MomentOfInertiaUnit PoundSquareInch = PoundSquareInches;
}
