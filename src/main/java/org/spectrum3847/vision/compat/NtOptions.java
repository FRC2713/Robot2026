package org.spectrum3847.vision.compat;

import edu.wpi.first.networktables.PubSubOption;

/** NetworkTables subscription options (WPILib 2026: PubSubOption.sendAll(boolean)). */
public final class NtOptions {
    private NtOptions() {}

    /** Every value, queued (up to {@code queue}), sampled every 10 ms: for per-frame data. */
    public static PubSubOption[] everyValue(int queue) {
        return new PubSubOption[] {PubSubOption.sendAll(true), PubSubOption.pollStorage(queue), PubSubOption.periodic(0.01)};
    }
}
