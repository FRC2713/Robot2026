package org.spectrum3847.vision.heading;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import java.util.Optional;

/**
 * The robot's recent heading, turn rate and speed, by time, so a frame captured 30 ms ago is
 * judged against the gyro as it was then (not now, when the robot has turned since).
 *
 * <p>Feed it every robot loop (or from a faster odometry thread) with timestamps in the same
 * timebase as PhotonLib's ({@code Timer.getTimestamp()}). CTRE swerve: its state timestamps are in
 * Phoenix's time, so convert ({@code Utils.currentTimeToFPGATime(state.Timestamp)}); see the README.
 */
public class RobotMotionHistory {
    private final TimeInterpolatableBuffer<Rotation2d> heading;
    private final TimeInterpolatableBuffer<Double> yawRate;
    private final TimeInterpolatableBuffer<Double> speed;
    private double latest = Double.NEGATIVE_INFINITY;

    /** @param historySeconds how far back to keep (frames older than this can't use the gyro) */
    public RobotMotionHistory(double historySeconds) {
        heading = TimeInterpolatableBuffer.createBuffer(historySeconds);
        yawRate = TimeInterpolatableBuffer.createDoubleBuffer(historySeconds);
        speed = TimeInterpolatableBuffer.createDoubleBuffer(historySeconds);
    }

    public RobotMotionHistory() {
        this(1.5);
    }

    /**
     * @param timestampSeconds when the sample was taken (robot timebase)
     * @param robotHeading field-relative heading (the one the pose estimator uses)
     * @param yawRateRadPerSec turn rate, rad/s (counter-clockwise positive)
     * @param linearSpeedMetersPerSec |robot velocity|, m/s
     */
    public synchronized void add(
            double timestampSeconds, Rotation2d robotHeading, double yawRateRadPerSec, double linearSpeedMetersPerSec) {
        heading.addSample(timestampSeconds, robotHeading);
        yawRate.addSample(timestampSeconds, yawRateRadPerSec);
        speed.addSample(timestampSeconds, linearSpeedMetersPerSec);
        latest = Math.max(latest, timestampSeconds);
    }

    /** The heading at {@code t}, if the history covers it. */
    public synchronized Optional<Rotation2d> headingAt(double t) {
        return covers(t) ? heading.getSample(t) : Optional.empty();
    }

    public synchronized Optional<Double> yawRateAt(double t) {
        return covers(t) ? yawRate.getSample(t) : Optional.empty();
    }

    public synchronized Optional<Double> speedAt(double t) {
        return covers(t) ? speed.getSample(t) : Optional.empty();
    }

    /** Newest sample time (-inf before the first). */
    public synchronized double latestTimestamp() {
        return latest;
    }

    // Outside the history WPILib's buffer returns its first or last sample: a frame older than the
    // history would be judged against the oldest heading. Allow 50 ms past the newest sample (a
    // frame captured just after this loop's gyro read), and nothing before the oldest.
    private boolean covers(double t) {
        var b = heading.getInternalBuffer();
        return !b.isEmpty() && t >= b.firstKey() && t <= latest + 0.05;
    }

    public synchronized void clear() {
        heading.clear();
        yawRate.clear();
        speed.clear();
        latest = Double.NEGATIVE_INFINITY;
    }
}
