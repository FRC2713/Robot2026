package org.spectrum3847.vision.gate;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.Set;
import org.spectrum3847.vision.heading.RobotMotionHistory;

/**
 * What gates can look at besides the candidate: the time now, the field, the gyro history, the pose
 * estimator's current estimate, whether the robot is enabled, and the tags to leave out.
 *
 * @param nowSeconds robot time now ({@code Timer.getTimestamp()})
 * @param field the tag layout
 * @param motion the gyro and speed history (never null; may be empty)
 * @param currentEstimate the pose estimator's estimate now, or null if not given
 * @param enabled whether the robot is enabled
 * @param excludedTags tags to leave out (yours, plus the Jetson's {@code excludedTagsActive})
 */
public record GateContext(
        double nowSeconds,
        AprilTagFieldLayout field,
        RobotMotionHistory motion,
        Pose2d currentEstimate,
        boolean enabled,
        Set<Integer> excludedTags) {}
