package org.spectrum3847.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.List;
import org.spectrum3847.vision.observation.FrameObservation;
import org.spectrum3847.vision.observation.PoseObservation;

/**
 * What one {@link VisionSystem#periodic()} did: every frame, every candidate pose, and the verdict
 * on each. Log what you need from it (the README lists what we log).
 *
 * @param nowSeconds robot time of the loop
 * @param frames every new frame with tags, all cameras
 * @param accepted candidates sent to the pose estimator, oldest first
 * @param rejected candidates a gate rejected
 */
public record VisionUpdate(
        double nowSeconds, List<FrameObservation> frames, List<Accepted> accepted, List<Rejected> rejected) {

    /** @param stdDevs x, y (m) and heading (rad) standard deviations sent with it */
    public record Accepted(PoseObservation observation, Matrix<N3, N1> stdDevs) {
        public Pose2d pose() {
            return observation.robotPose().toPose2d();
        }
    }

    /** @param gate the gate that rejected it; @param reason why */
    public record Rejected(PoseObservation observation, String gate, String reason) {}

    public List<Pose2d> acceptedPoses() {
        return accepted.stream().map(Accepted::pose).toList();
    }

    public List<Pose2d> rejectedPoses() {
        return rejected.stream().map(r -> r.observation().robotPose().toPose2d()).toList();
    }

    /** The newest accepted pose, or null. */
    public Accepted latestAccepted() {
        return accepted.isEmpty() ? null : accepted.get(accepted.size() - 1);
    }
}
