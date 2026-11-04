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

import static frc2713.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  private final VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOPhotonVisionSim(
      String name,
      Transform3d robotToCamera,
      Supplier<Pose2d> poseSupplier,
      Supplier<Pose2d> estimatedPoseSupplier) {
    super(name, robotToCamera, estimatedPoseSupplier);
    this.poseSupplier = poseSupplier;

    // Initialize vision sim
    visionSim = new VisionSystemSim(name);
    visionSim.addAprilTags(aprilTagLayout);

    // Add sim camera
    var cameraProperties = new SimCameraProperties();
    cameraProperties.setCalibration(1600, 1200, Rotation2d.fromDegrees(90));
    cameraProperties.setCalibError(0.4, 0.10);
    cameraProperties.setFPS(25);
    cameraProperties.setAvgLatencyMs(50);
    cameraProperties.setLatencyStdDevMs(15);

    cameraSim = new PhotonCameraSim(camera, cameraProperties);

    // Pose simulation does not need video rendering.
    cameraSim.enableDrawWireframe(false);
    cameraSim.enableRawStream(false);
    cameraSim.enableProcessedStream(false);
    cameraSim.setMaxSightRange(Units.feetToMeters(22.0));

    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }

  @Override
  public void close() {
    visionSim.removeCamera(cameraSim);
    cameraSim.close();
    super.close();
  }
}
