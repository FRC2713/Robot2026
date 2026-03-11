package frc2713.lib.logging;

import java.lang.reflect.Method;
import org.littletonrobotics.junction.Logger;

/** Times annotated periodic methods and logs lightweight statistics. */
public final class PeriodicTimingLogger {
  private static final TimingScope NOOP_SCOPE = () -> {};
  private static final double NANOS_TO_MILLIS = 1.0e-6;
  private static final double EWMA_ALPHA = 0.1;

  private static final ClassValue<TimerMetadata> METADATA_CACHE =
      new ClassValue<>() {
        @Override
        protected TimerMetadata computeValue(Class<?> type) {
          return createMetadata(type);
        }
      };

  private PeriodicTimingLogger() {}

  /**
   * Starts a timing scope for the instance's periodic method. Returns a no-op scope if the periodic
   * method is not annotated with {@link TimeLogged}.
   */
  public static TimingScope time(Object instance) {
    TimerMetadata metadata = METADATA_CACHE.get(instance.getClass());
    if (!metadata.enabled) {
      return NOOP_SCOPE;
    }
    return new ActiveTimingScope(metadata);
  }

  /** Scope used by try-with-resources to guarantee timing closeout. */
  public interface TimingScope extends AutoCloseable {
    @Override
    void close();
  }

  private static TimerMetadata createMetadata(Class<?> type) {
    final Method periodicMethod;
    try {
      periodicMethod = type.getMethod("periodic");
    } catch (NoSuchMethodException ignored) {
      return TimerMetadata.disabled();
    }

    TimeLogged annotation = periodicMethod.getAnnotation(TimeLogged.class);
    if (annotation == null) {
      return TimerMetadata.disabled();
    }

    String path = sanitizePath(annotation.value());
    if (path.isEmpty()) {
      return TimerMetadata.disabled();
    }
    return TimerMetadata.enabled(path);
  }

  private static String sanitizePath(String path) {
    String trimmed = path.trim();
    while (trimmed.endsWith("/")) {
      trimmed = trimmed.substring(0, trimmed.length() - 1);
    }
    return trimmed;
  }

  private static final class ActiveTimingScope implements TimingScope {
    private final TimerMetadata metadata;
    private final long startNanos;

    private ActiveTimingScope(TimerMetadata metadata) {
      this.metadata = metadata;
      this.startNanos = System.nanoTime();
    }

    @Override
    public void close() {
      double elapsedMs = (System.nanoTime() - startNanos) * NANOS_TO_MILLIS;
      StatsSnapshot snapshot = metadata.stats.addSample(elapsedMs);

      Logger.recordOutput(metadata.path + "/lastMs", snapshot.lastMs);
      Logger.recordOutput(metadata.path + "/avgMs", snapshot.avgMs);
      Logger.recordOutput(metadata.path + "/minMs", snapshot.minMs);
      Logger.recordOutput(metadata.path + "/maxMs", snapshot.maxMs);
      Logger.recordOutput(metadata.path + "/ewmaMs", snapshot.ewmaMs);
      Logger.recordOutput(metadata.path + "/samples", (double) snapshot.samples);
    }
  }

  private static final class TimerMetadata {
    private final boolean enabled;
    private final String path;
    private final RunningStats stats;

    private TimerMetadata(boolean enabled, String path, RunningStats stats) {
      this.enabled = enabled;
      this.path = path;
      this.stats = stats;
    }

    private static TimerMetadata enabled(String path) {
      return new TimerMetadata(true, path, new RunningStats());
    }

    private static TimerMetadata disabled() {
      return new TimerMetadata(false, "", null);
    }
  }

  private static final class RunningStats {
    private long samples = 0;
    private double avgMs = 0.0;
    private double minMs = Double.POSITIVE_INFINITY;
    private double maxMs = Double.NEGATIVE_INFINITY;
    private double ewmaMs = 0.0;
    private boolean ewmaInitialized = false;

    private synchronized StatsSnapshot addSample(double sampleMs) {
      samples++;
      avgMs += (sampleMs - avgMs) / samples;
      minMs = Math.min(minMs, sampleMs);
      maxMs = Math.max(maxMs, sampleMs);

      if (!ewmaInitialized) {
        ewmaMs = sampleMs;
        ewmaInitialized = true;
      } else {
        ewmaMs = (EWMA_ALPHA * sampleMs) + ((1.0 - EWMA_ALPHA) * ewmaMs);
      }

      return new StatsSnapshot(samples, sampleMs, avgMs, minMs, maxMs, ewmaMs);
    }
  }

  private record StatsSnapshot(
      long samples, double lastMs, double avgMs, double minMs, double maxMs, double ewmaMs) {}
}
