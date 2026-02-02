package frc2713.lib.util;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import java.util.function.Supplier;

/**
 * A wrapper for {@link LoggedTunableNumber} that handles WPILib Units.
 *
 * @param <M> The type of Measure being wrapped.
 */
public class LoggedTunableMeasure<M extends Measure<?>> implements Supplier<M> {
  private final LoggedTunableNumber tunableNumber;
  private final Unit unit;

  public LoggedTunableMeasure(String path, M defaultValue) {
    this.unit = defaultValue.unit();
    this.tunableNumber =
        new LoggedTunableNumber(path + " " + unit.name(), defaultValue.magnitude());
  }

  @Override
  @SuppressWarnings("unchecked")
  public M get() {
    return (M) unit.of(tunableNumber.get());
  }
}
