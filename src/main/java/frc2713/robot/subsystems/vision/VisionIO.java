// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc2713.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

/** Camera acquisition only. Vision owns validation and estimator updates. */
public interface VisionIO extends AutoCloseable {
  @AutoLog
  class VisionIOInputs {
    public boolean connected;
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
    public int frameCount;
    public long qualityMatchedFrames;
    public long qualityUnmatchedFrames;
    public TagQuality[] tagQuality = new TagQuality[0];
  }

  /** Tag count and distance describe only the targets actually used for this solve. */
  record PoseObservation(
      double timestamp, Pose3d pose, double ambiguity, int tagCount, double averageTagDistance) {}

  /** Optional Jetson diagnostics, matched to a PhotonVision frame and tag. */
  record TagQuality(
      long sequenceId,
      double timestamp,
      int tagId,
      double decisionMargin,
      double edgePx,
      double undistortPx,
      double reprojBestPx,
      double reprojAltPx) {}

  default void updateInputs(VisionIOInputs inputs) {}

  @Override
  default void close() {}
}
