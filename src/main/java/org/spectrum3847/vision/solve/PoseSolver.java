package org.spectrum3847.vision.solve;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.List;
import org.spectrum3847.vision.heading.RobotMotionHistory;
import org.spectrum3847.vision.observation.FrameObservation;
import org.spectrum3847.vision.observation.PoseObservation;

/**
 * Turns one camera frame into robot pose candidates. Candidates are judged afterwards by the gates
 * and given a trust by the trust model; a solver only does geometry.
 */
@FunctionalInterface
public interface PoseSolver {
    /**
     * @param frame the frame
     * @param robotToCamera where the camera is on the robot
     * @param field the tag layout (must match the coprocessor's for multi-tag)
     * @param motion the gyro history, for solvers that use the heading (may be empty)
     * @return zero or more candidates
     */
    List<PoseObservation> solve(
            FrameObservation frame, Transform3d robotToCamera, AprilTagFieldLayout field, RobotMotionHistory motion);
}
