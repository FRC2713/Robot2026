package org.spectrum3847.vision.sink;

import java.util.function.DoubleSupplier;
import org.spectrum3847.vision.compat.Clock;

/**
 * Converts timestamps between robot time (PhotonLib's and {@code Timer.getTimestamp()}'s) and
 * another clock, by reading both clocks at the moment of the conversion: no assumption about either
 * clock's epoch. For CTRE Phoenix 6, whose swerve takes timestamps in {@code
 * Utils.getCurrentTimeSeconds()}'s epoch: Phoenix 2026 has {@code Utils.fpgaToCurrentTime}, but
 * Phoenix 2027 (26.50 alpha) dropped it and asks you to "sync the epochs"; this does that, and
 * works on both.
 *
 * <pre>{@code
 * var phoenix = new ClockBridge(Utils::getCurrentTimeSeconds);
 * .sink(PoseSink.convertingTime(drivetrain::addVisionMeasurement, phoenix::toOther))
 * drivetrain.registerTelemetry(s -> vision.addMotion(phoenix.fromOther(s.Timestamp), ...));
 * }</pre>
 *
 * Both clocks must run at the same rate (they do: both count seconds of the same system), so the
 * offset is constant apart from the microseconds between the two reads.
 */
public final class ClockBridge {
    private final DoubleSupplier other;
    private final DoubleSupplier robot;

    /** @param otherNow the other clock now, seconds */
    public ClockBridge(DoubleSupplier otherNow) {
        this(otherNow, Clock::now);
    }

    public ClockBridge(DoubleSupplier otherNow, DoubleSupplier robotNow) {
        this.other = otherNow;
        this.robot = robotNow;
    }

    /** other - robot, seconds, now. */
    public double offsetSeconds() {
        return other.getAsDouble() - robot.getAsDouble();
    }

    /** A robot-time timestamp in the other clock. */
    public double toOther(double robotSeconds) {
        return robotSeconds + offsetSeconds();
    }

    /** An other-clock timestamp in robot time. */
    public double fromOther(double otherSeconds) {
        return otherSeconds - offsetSeconds();
    }
}
