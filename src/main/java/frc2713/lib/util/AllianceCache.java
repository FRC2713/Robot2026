package frc2713.lib.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/** Caches alliance data to avoid repeated DriverStation polling in hot paths. */
public final class AllianceCache {
  private static final double DISABLED_REFRESH_PERIOD_SEC = 0.5;

  private static Optional<Alliance> cachedAlliance = Optional.empty();
  private static double lastDisabledRefreshTimeSec = Double.NEGATIVE_INFINITY;
  private static double lastAllianceChangeTimeSec = Double.NEGATIVE_INFINITY;

  private AllianceCache() {}

  /** Polls until alliance is known, then only serves cache in normal periodic loops. */
  public static void periodicUpdate() {
    if (cachedAlliance.isEmpty()) {
      refreshFromDriverStation();
    }
    logState();
  }

  /**
   * Refreshes alliance while disabled on a throttled interval.
   *
   * <p>This supports changing alliance on the driver station and then disabling to apply it.
   */
  public static void disabledPeriodicUpdate() {
    double nowSec = Timer.getFPGATimestamp();
    if (nowSec - lastDisabledRefreshTimeSec < DISABLED_REFRESH_PERIOD_SEC) {
      logState();
      return;
    }
    lastDisabledRefreshTimeSec = nowSec;
    refreshFromDriverStation();
    logState();
  }

  /** Immediately checks the driver station once and updates cache/logs. */
  public static void refreshNow() {
    refreshFromDriverStation();
    logState();
  }

  public static boolean hasAlliance() {
    return cachedAlliance.isPresent();
  }

  public static Optional<Alliance> getAlliance() {
    return cachedAlliance;
  }

  public static Alliance getAllianceOrBlue() {
    return cachedAlliance.orElse(Alliance.Blue);
  }

  public static boolean isRed() {
    return getAllianceOrBlue() == Alliance.Red;
  }

  private static void refreshFromDriverStation() {
    Optional<Alliance> currentAlliance = DriverStation.getAlliance();
    if (currentAlliance.isPresent() && !currentAlliance.equals(cachedAlliance)) {
      cachedAlliance = currentAlliance;
      lastAllianceChangeTimeSec = Timer.getFPGATimestamp();
    }
  }

  private static void logState() {
    Logger.recordOutput("AllianceCache/HasAlliance", cachedAlliance.isPresent());
    Logger.recordOutput("AllianceCache/IsRed", isRed());
    Logger.recordOutput(
        "AllianceCache/Alliance",
        cachedAlliance.map(Alliance::name).orElse("Unknown"));
    Logger.recordOutput("AllianceCache/LastAllianceChangeTimeSec", lastAllianceChangeTimeSec);
    Logger.recordOutput("AllianceCache/LastDisabledRefreshTimeSec", lastDisabledRefreshTimeSec);
  }
}
