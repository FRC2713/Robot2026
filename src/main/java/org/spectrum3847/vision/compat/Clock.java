package org.spectrum3847.vision.compat;

import edu.wpi.first.wpilibj.Timer;

/** Robot time in seconds, the timebase of PhotonLib's timestamps (WPILib 2026 build). */
public final class Clock {
    private Clock() {}

    public static double now() {
        return Timer.getTimestamp();
    }
}
