package frc2713.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc2713.robot.Constants;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PnpResult;
import org.photonvision.targeting.TargetCorner;

class VisionIOPhotonVisionTest {
  @BeforeAll
  static void initialize() {
    assertTrue(HAL.initialize(500, 0));
    Constants.disableHAL();
  }

  private static PhotonPoseEstimator estimator() {
    return new PhotonPoseEstimator(
        new AprilTagFieldLayout(
            List.of(
                new AprilTag(1, new Pose3d(5, 2, 0, Rotation3d.kZero)),
                new AprilTag(2, new Pose3d(5, 4, 0, Rotation3d.kZero)),
                new AprilTag(3, new Pose3d(6, 4, 0, Rotation3d.kZero))),
            16,
            8),
        Transform3d.kZero);
  }

  private static PhotonTrackedTarget target(int id, double ambiguity) {
    var corners =
        List.of(
            new TargetCorner(0, 0),
            new TargetCorner(1, 0),
            new TargetCorner(1, 1),
            new TargetCorner(0, 1));
    return new PhotonTrackedTarget(
        0,
        0,
        1,
        0,
        id,
        -1,
        0,
        new Transform3d(2, 0, 0, Rotation3d.kZero),
        new Transform3d(4, 0, 0, Rotation3d.kZero),
        ambiguity,
        corners,
        corners);
  }

  @Test
  void fallbackUsesReferencePoseToChooseAlternateSolutionAndReportsOneTag() {
    var frame =
        new PhotonPipelineResult(
            1, 1_000_000, 1_050_000, 0, List.of(target(1, 0.2), target(2, 0.1)));
    var reference = new Pose2d(1, 2, new edu.wpi.first.math.geometry.Rotation2d());
    var estimate = VisionIOPhotonVision.estimatePose(estimator(), frame, reference).orElseThrow();
    assertEquals(reference, estimate.estimatedPose.toPose2d());
    assertEquals(1, estimate.targetsUsed.size(), "Fallback is single-tag even with two detections");
    assertEquals(1, estimate.targetsUsed.get(0).getFiducialId());
    assertEquals(0.2, estimate.targetsUsed.get(0).getPoseAmbiguity());
    assertEquals(1.0, estimate.timestampSeconds);
  }

  @Test
  void multitagUsesCoprocessorPoseAndOnlyTagsIncludedInTheSolve() {
    var pose = new Transform3d(3, 2, 0, Rotation3d.kZero);
    var multi = new MultiTargetPNPResult(new PnpResult(pose, 0.1), List.of((short) 1, (short) 2));
    var frame =
        new PhotonPipelineResult(
            1,
            1_000_000,
            1_050_000,
            0,
            List.of(target(1, 0.2), target(2, 0.1), target(3, 0.9)),
            Optional.of(multi));
    var estimate =
        VisionIOPhotonVision.estimatePose(estimator(), frame, Pose2d.kZero).orElseThrow();
    assertEquals(new Pose3d().plus(pose), estimate.estimatedPose);
    assertEquals(
        List.of(1, 2),
        estimate.targetsUsed.stream().map(PhotonTrackedTarget::getFiducialId).toList());
  }

  @Test
  void noTargetsOrUnknownTagsProduceNoEstimate() {
    assertTrue(
        VisionIOPhotonVision.estimatePose(estimator(), new PhotonPipelineResult(), Pose2d.kZero)
            .isEmpty());
    assertTrue(
        VisionIOPhotonVision.estimatePose(
                estimator(),
                new PhotonPipelineResult(1, 1_000_000, 1_050_000, 0, List.of(target(999, 0.1))),
                Pose2d.kZero)
            .isEmpty());
  }

  @Test
  void simulatedCameraPublishesUsableObservationsWithoutHardware() {
    var tag = VisionConstants.aprilTagLayout.getTagPose(1).orElseThrow();
    var robotPose =
        tag.transformBy(new Transform3d(2, 0, 0, new Rotation3d(0, 0, Math.PI))).toPose2d();
    var robotToCamera = new Transform3d(0, 0, tag.getZ(), Rotation3d.kZero);
    SimHooks.pauseTiming();
    try (var io =
        new VisionIOPhotonVisionSim(
            "vision-regression-sim", robotToCamera, () -> robotPose, () -> robotPose)) {
      var inputs = new VisionIO.VisionIOInputs();
      int usableObservations = 0;
      for (int i = 0; i < 100; i++) {
        SimHooks.stepTiming(0.02);
        io.updateInputs(inputs);
        for (var observation : inputs.poseObservations) {
          if (!Vision.shouldRejectPoseObservation(observation)) {
            usableObservations++;
            assertTrue(
                observation
                        .pose()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(robotPose.getTranslation())
                    < 0.5);
          }
        }
      }
      assertTrue(
          usableObservations > 0, "Sim must generate accepted camera poses without a coprocessor");
    } finally {
      SimHooks.resumeTiming();
    }
  }
}
