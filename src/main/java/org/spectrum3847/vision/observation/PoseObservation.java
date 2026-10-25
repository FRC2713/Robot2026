package org.spectrum3847.vision.observation;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.List;

/**
 * A robot pose worked out from one camera frame (by a {@link org.spectrum3847.vision.solve.PoseSolver}),
 * with what the gates and the trust model judge it by.
 *
 * @param robotPose the robot's pose on the field (3D: height, roll and pitch let gates catch a bad
 *     solve)
 * @param timestampSeconds capture time, robot timebase
 * @param type how it was solved
 * @param tagIds the tags it rests on
 * @param averageDistanceMeters mean camera-to-tag distance
 * @param minDistanceMeters nearest tag
 * @param ambiguity the single tag's ambiguity (0 for multi-tag; -1 unknown)
 * @param reprojErrPx RMS reprojection error behind it: multi-tag's, or the single tag's best (NaN
 *     if unknown)
 * @param minEdgePx the nearest corner to the image edge over its tags (NaN if unknown)
 * @param minDecisionMargin the lowest decision margin over its tags (NaN if unknown)
 * @param headingDisagreementRad |vision yaw - gyro yaw| at capture, radians (NaN without a gyro)
 * @param frame the frame it came from
 */
public record PoseObservation(
        Pose3d robotPose,
        double timestampSeconds,
        Type type,
        List<Integer> tagIds,
        double averageDistanceMeters,
        double minDistanceMeters,
        double ambiguity,
        double reprojErrPx,
        double minEdgePx,
        double minDecisionMargin,
        double headingDisagreementRad,
        FrameObservation frame) {

    public PoseObservation {
        tagIds = List.copyOf(tagIds);
    }

    public int tagCount() {
        return tagIds.size();
    }

    public String camera() {
        return frame.camera();
    }

    /** How a pose was solved, best to worst. */
    public enum Type {
        /** PhotonVision's multi-tag solve: every corner of every tag in one fit. Full pose. */
        MULTI_TAG,
        /** One tag, its solutions clearly different (low ambiguity): the best one. Full pose. */
        SINGLE_TAG,
        /** One tag, ambiguous: the solution whose heading agrees with the gyro. Full pose. */
        SINGLE_TAG_GYRO,
        /**
         * One tag's position (not its orientation) plus the gyro heading: robust to single-tag
         * ambiguity, and the most accurate single-tag translation at range. Heading is the gyro's.
         */
        TAG_TRIG
    }

    /** True if the heading comes from the camera (and may correct the gyro). */
    public boolean headingFromVision() {
        return type == Type.MULTI_TAG || type == Type.SINGLE_TAG || type == Type.SINGLE_TAG_GYRO;
    }
}
