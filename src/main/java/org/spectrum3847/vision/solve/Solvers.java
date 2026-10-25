package org.spectrum3847.vision.solve;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.spectrum3847.vision.heading.RobotMotionHistory;
import org.spectrum3847.vision.observation.FrameObservation;
import org.spectrum3847.vision.observation.PoseObservation;
import org.spectrum3847.vision.observation.TagObservation;

/**
 * The standard solvers, and {@link #standard()}: what we recommend, in one call.
 *
 * <p>Conventions follow PhotonLib: {@code robotToCamera} is the camera's pose on the robot (camera
 * frame x forward, y left, z up); {@code cameraToTag} is the tag's pose in the camera frame; field
 * tag poses come from the layout. So {@code fieldToRobot = fieldToTag * cameraToTag^-1 *
 * robotToCamera^-1}.
 */
public final class Solvers {
    private Solvers() {}

    /**
     * The recommended solver:
     *
     * <ul>
     *   <li>a frame with PhotonVision's multi-tag result: that, one candidate ({@link
     *       PoseObservation.Type#MULTI_TAG}),
     *   <li>otherwise each tag on its own: with the gyro, {@link #tagTrig} (the tag's position plus
     *       the gyro heading, which single-tag ambiguity can't fool); without it, the best solution
     *       if it's unambiguous.
     * </ul>
     */
    public static PoseSolver standard() {
        return multiTagThen(singleTag(new SingleTagConfig()));
    }

    /** PhotonVision's multi-tag result if the frame has one, otherwise {@code fallback}. */
    public static PoseSolver multiTagThen(PoseSolver fallback) {
        return (frame, robotToCamera, field, motion) ->
                frame.hasMultiTag() && frame.multiTagIds().size() >= 2
                        ? multiTag().solve(frame, robotToCamera, field, motion)
                        : fallback.solve(frame, robotToCamera, field, motion);
    }

    /** PhotonVision's multi-tag solve, moved from the camera to the robot. */
    public static PoseSolver multiTag() {
        return (frame, robotToCamera, field, motion) -> {
            if (!frame.hasMultiTag()) return List.of();
            Pose3d robot = new Pose3d().plus(frame.multiTagFieldToCamera()).plus(robotToCamera.inverse());
            var tags = frame.tags().stream().filter(t -> frame.multiTagIds().contains(t.id())).toList();
            return List.of(candidate(robot, frame, PoseObservation.Type.MULTI_TAG, tags, 0, frame.multiTagReprojErrPx(), motion));
        };
    }

    /** Settings for the single-tag solvers. */
    public static final class SingleTagConfig {
        /** Below this ambiguity the best solution is trusted as it is. PhotonLib's default 0.2. */
        public double unambiguousBelow = 0.2;
        /** Above this nothing is solved from the full pose (the gyro trig solve still can be). */
        public double maxAmbiguity = 0.9;
        /** Gyro disambiguation: the chosen solution's heading must be this close to the gyro. */
        public double maxHeadingErrorRad = Math.toRadians(15);
        /** Use the gyro trig solve when a gyro sample covers the frame (recommended). */
        public boolean preferTrig = true;
        /** Also give the full-pose candidate when the trig solve is used (normally off). */
        public boolean alsoFullPose = false;

        public SingleTagConfig unambiguousBelow(double v) {
            unambiguousBelow = v;
            return this;
        }

        public SingleTagConfig maxAmbiguity(double v) {
            maxAmbiguity = v;
            return this;
        }

        public SingleTagConfig maxHeadingErrorDeg(double deg) {
            maxHeadingErrorRad = Math.toRadians(deg);
            return this;
        }

        public SingleTagConfig preferTrig(boolean v) {
            preferTrig = v;
            return this;
        }

        public SingleTagConfig alsoFullPose(boolean v) {
            alsoFullPose = v;
            return this;
        }
    }

    /** Every tag on its own (see {@link SingleTagConfig}). */
    public static PoseSolver singleTag(SingleTagConfig cfg) {
        return (frame, robotToCamera, field, motion) -> {
            var out = new ArrayList<PoseObservation>();
            Optional<Rotation2d> gyro = motion == null ? Optional.empty() : motion.headingAt(frame.timestampSeconds());
            for (var tag : frame.tags()) {
                var tagPose = field.getTagPose(tag.id());
                if (tagPose.isEmpty()) continue;
                boolean trig = cfg.preferTrig && gyro.isPresent();
                if (trig) out.add(trig(frame, tag, tagPose.get(), robotToCamera, gyro.get(), motion));
                if (!trig || cfg.alsoFullPose) {
                    var full = fullPose(frame, tag, tagPose.get(), robotToCamera, gyro, cfg, motion);
                    if (full != null) out.add(full);
                }
            }
            return out;
        };
    }

    /** The tag's position plus the gyro heading, for every tag (frames without a gyro sample give nothing). */
    public static PoseSolver tagTrig() {
        return (frame, robotToCamera, field, motion) -> {
            var out = new ArrayList<PoseObservation>();
            if (motion == null) return out;
            var gyro = motion.headingAt(frame.timestampSeconds());
            if (gyro.isEmpty()) return out;
            for (var tag : frame.tags()) {
                var tagPose = field.getTagPose(tag.id());
                if (tagPose.isPresent()) out.add(trig(frame, tag, tagPose.get(), robotToCamera, gyro.get(), motion));
            }
            return out;
        };
    }

    /** The robot pose from one of a tag's two solutions. */
    public static Pose3d robotFromTag(Pose3d fieldToTag, Transform3d cameraToTag, Transform3d robotToCamera) {
        return fieldToTag.transformBy(cameraToTag.inverse()).transformBy(robotToCamera.inverse());
    }

    private static PoseObservation fullPose(
            FrameObservation frame,
            TagObservation tag,
            Pose3d tagPose,
            Transform3d robotToCamera,
            Optional<Rotation2d> gyro,
            SingleTagConfig cfg,
            RobotMotionHistory motion) {
        double amb = tag.ambiguity();
        if (amb > cfg.maxAmbiguity) return null;
        Pose3d best = robotFromTag(tagPose, tag.bestCameraToTag(), robotToCamera);
        if (amb >= 0 && amb < cfg.unambiguousBelow) {
            return candidate(best, frame, PoseObservation.Type.SINGLE_TAG, List.of(tag), amb, tag.reprojBestPx(), motion);
        }
        // Ambiguous: of the two solutions, the one that agrees with the gyro, if either does.
        if (gyro.isEmpty()) return null;
        Pose3d alt = robotFromTag(tagPose, tag.altCameraToTag(), robotToCamera);
        double eBest = Math.abs(MathUtil.angleModulus(best.getRotation().getZ() - gyro.get().getRadians()));
        double eAlt = Math.abs(MathUtil.angleModulus(alt.getRotation().getZ() - gyro.get().getRadians()));
        boolean useAlt = eAlt < eBest;
        if (Math.min(eBest, eAlt) > cfg.maxHeadingErrorRad) return null;
        return candidate(
                useAlt ? alt : best,
                frame,
                PoseObservation.Type.SINGLE_TAG_GYRO,
                List.of(tag),
                amb,
                useAlt ? tag.reprojAltPx() : tag.reprojBestPx(),
                motion);
    }

    /**
     * The gyro trig solve: the tag's position in the camera frame (from the best solution: both
     * solutions agree on position, they differ in orientation), turned into the field frame with
     * the gyro heading and the camera's mount. The robot is taken as flat (roll and pitch 0).
     */
    static PoseObservation trig(
            FrameObservation frame,
            TagObservation tag,
            Pose3d tagPose,
            Transform3d robotToCamera,
            Rotation2d heading,
            RobotMotionHistory motion) {
        // Tag position in the robot frame.
        Translation3d inRobot =
                tag.bestCameraToTag().getTranslation().rotateBy(robotToCamera.getRotation()).plus(robotToCamera.getTranslation());
        // ...and in the field frame's orientation, with the gyro heading.
        Translation3d inField = inRobot.rotateBy(new Rotation3d(0, 0, heading.getRadians()));
        Translation3d robot = tagPose.getTranslation().minus(inField);
        var pose = new Pose3d(new Translation3d(robot.getX(), robot.getY(), 0), new Rotation3d(0, 0, heading.getRadians()));
        return candidate(pose, frame, PoseObservation.Type.TAG_TRIG, List.of(tag), tag.ambiguity(), tag.reprojBestPx(), motion);
    }

    static PoseObservation candidate(
            Pose3d robot,
            FrameObservation frame,
            PoseObservation.Type type,
            List<TagObservation> tags,
            double ambiguity,
            double reprojErrPx,
            RobotMotionHistory motion) {
        double sum = 0, min = Double.POSITIVE_INFINITY, edge = Double.NaN, margin = Double.NaN;
        var ids = new ArrayList<Integer>();
        for (var t : tags) {
            double d = t.distanceMeters();
            sum += d;
            min = Math.min(min, d);
            ids.add(t.id());
            if (!Double.isNaN(t.edgePx())) edge = Double.isNaN(edge) ? t.edgePx() : Math.min(edge, t.edgePx());
            if (!Double.isNaN(t.decisionMargin()))
                margin = Double.isNaN(margin) ? t.decisionMargin() : Math.min(margin, t.decisionMargin());
        }
        if (tags.isEmpty()) min = Double.NaN;
        double disagreement = Double.NaN;
        if (motion != null) {
            var gyro = motion.headingAt(frame.timestampSeconds());
            if (gyro.isPresent()) {
                disagreement = Math.abs(MathUtil.angleModulus(robot.getRotation().getZ() - gyro.get().getRadians()));
            }
        }
        return new PoseObservation(
                robot,
                frame.timestampSeconds(),
                type,
                ids,
                tags.isEmpty() ? Double.NaN : sum / tags.size(),
                min,
                ambiguity,
                reprojErrPx,
                edge,
                margin,
                disagreement,
                frame);
    }
}
