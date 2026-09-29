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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

/** PhotonVision 2026 multitag estimation with closest-reference single-tag fallback. */
public class VisionIOPhotonVision implements VisionIO, AutoCloseable {
  protected final PhotonCamera camera;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final PhotonPoseEstimator poseEstimator;

  public VisionIOPhotonVision(
      String name, Transform3d robotToCamera, Supplier<Pose2d> robotPoseSupplier) {
    camera = new PhotonCamera(name);
    this.robotPoseSupplier = robotPoseSupplier;
    poseEstimator = new PhotonPoseEstimator(aprilTagLayout, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();
    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> observations = new ArrayList<>();
    var referencePose = robotPoseSupplier.get();

    // Drain every frame once; PhotonLib timestamps are synchronized to the robot clock.
    for (var result : camera.getAllUnreadResults()) {
      inputs.latestTargetObservation =
          result.hasTargets()
              ? new TargetObservation(
                  Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                  Rotation2d.fromDegrees(result.getBestTarget().getPitch()))
              : new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
      for (var target : result.getTargets()) {
        if (aprilTagLayout.getTagPose(target.getFiducialId()).isPresent()) {
          tagIds.add(target.getFiducialId());
        }
      }

      estimatePose(poseEstimator, result, referencePose)
          .ifPresent(
              estimate -> {
                var targets = estimate.targetsUsed;
                if (targets.isEmpty()) return;
                double distance =
                    targets.stream()
                        .mapToDouble(
                            target -> target.getBestCameraToTarget().getTranslation().getNorm())
                        .average()
                        .orElse(0.0);
                observations.add(
                    new PoseObservation(
                        estimate.timestampSeconds,
                        estimate.estimatedPose,
                        targets.size() == 1 ? targets.get(0).getPoseAmbiguity() : 0.0,
                        targets.size(),
                        distance,
                        PoseObservationType.PHOTONVISION));
              });
    }
    inputs.poseObservations = observations.toArray(PoseObservation[]::new);
    inputs.tagIds = tagIds.stream().mapToInt(Integer::intValue).sorted().toArray();
  }

  static Optional<EstimatedRobotPose> estimatePose(
      PhotonPoseEstimator estimator, PhotonPipelineResult result, Pose2d referencePose) {
    if (result.getMultiTagResult().isPresent()) {
      var ids = result.getMultiTagResult().get().fiducialIDsUsed;
      var usedTargets =
          result.getTargets().stream()
              .filter(target -> ids.contains((short) target.getFiducialId()))
              .filter(
                  target -> estimator.getFieldTags().getTagPose(target.getFiducialId()).isPresent())
              .toList();
      if (usedTargets.size() >= 2) {
        var estimate =
            estimator.estimateCoprocMultiTagPose(
                new PhotonPipelineResult(result.metadata, usedTargets, result.getMultiTagResult()));
        if (estimate.isPresent()) return estimate;
      }
    }

    // Evaluate one target at a time so targetsUsed describes the actual single-tag solve.
    // PhotonLib otherwise reports all visible targets for the reference-pose strategy,
    // which would incorrectly bypass single-tag rejection and heading suppression.
    Optional<EstimatedRobotPose> closest = Optional.empty();
    double smallestDistance = Double.POSITIVE_INFINITY;
    var reference = new Pose3d(referencePose);
    for (var target : result.getTargets()) {
      if (estimator.getFieldTags().getTagPose(target.getFiducialId()).isEmpty()) continue;
      var estimate =
          estimator.estimateClosestToReferencePose(
              new PhotonPipelineResult(result.metadata, List.of(target), Optional.empty()),
              reference);
      if (estimate.isPresent()) {
        double distance =
            estimate.get().estimatedPose.getTranslation().getDistance(reference.getTranslation());
        if (distance < smallestDistance) {
          smallestDistance = distance;
          closest = estimate;
        }
      }
    }
    return closest;
  }

  @Override
  public void close() {
    camera.close();
  }
}
