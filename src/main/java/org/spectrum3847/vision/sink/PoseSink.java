package org.spectrum3847.vision.sink;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.List;
import java.util.function.DoubleUnaryOperator;

/**
 * Where accepted vision measurements go. {@link org.spectrum3847.vision.VisionSystem} sends them in
 * capture-time order across all cameras, oldest first: WPILib's estimator discards a measurement
 * older than one it already applied, so order matters.
 *
 * <ul>
 *   <li>WPILib ({@code SwerveDrivePoseEstimator} and the others): {@link #wpilib(PoseEstimator)}.
 *   <li>CTRE swerve (Phoenix 6): its timestamps are in Phoenix's timebase, so convert with a
 *       {@link ClockBridge} (works with Phoenix 2026 and 2027):
 *       <pre>{@code
 * var phoenix = new ClockBridge(Utils::getCurrentTimeSeconds);
 * PoseSink.convertingTime(drivetrain::addVisionMeasurement, phoenix::toOther)
 * }</pre>
 *   <li>Several at once (WPILib's estimator and an EKF side by side, to compare): {@link #all}.
 * </ul>
 */
@FunctionalInterface
public interface PoseSink {
    /** Same signature as WPILib's and CTRE's {@code addVisionMeasurement}. */
    void addVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs);

    static PoseSink wpilib(PoseEstimator<?> estimator) {
        return estimator::addVisionMeasurement;
    }

    /** {@code sink} with each timestamp converted first (robot time to the sink's). */
    static PoseSink convertingTime(PoseSink sink, DoubleUnaryOperator robotToSinkTime) {
        return (pose, t, std) -> sink.addVisionMeasurement(pose, robotToSinkTime.applyAsDouble(t), std);
    }

    /** Every measurement to each of these. */
    static PoseSink all(PoseSink... sinks) {
        var list = List.of(sinks);
        return (pose, t, std) -> {
            for (var s : list) s.addVisionMeasurement(pose, t, std);
        };
    }
}
