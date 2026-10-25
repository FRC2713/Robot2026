package org.spectrum3847.vision.log;

import org.spectrum3847.vision.VisionUpdate;

/**
 * Gets every loop's {@link VisionUpdate}. {@link NetworkTablesVisionLogger} (the default) publishes
 * a compact summary for dashboards and AdvantageScope; with AdvantageKit, log from your own logger
 * instead (README, "Logging"), and pass {@link #NONE}.
 */
@FunctionalInterface
public interface VisionLogger {
    void log(VisionUpdate update, VisionStats stats);

    VisionLogger NONE = (u, s) -> {};
}
