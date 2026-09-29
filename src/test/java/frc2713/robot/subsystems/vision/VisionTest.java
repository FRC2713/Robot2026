package frc2713.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc2713.robot.Constants;
import frc2713.robot.subsystems.vision.VisionIO.PoseObservation;
import frc2713.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;

class VisionTest {
  @BeforeAll
  static void initialize() {
    assertTrue(HAL.initialize(500, 0));
    Constants.disableHAL();
  }

  private static PoseObservation observation(
      double x, double y, double z, int tags, double ambiguity, double distance) {
    return new PoseObservation(
        1.0,
        new Pose3d(x, y, z, Rotation3d.kZero),
        ambiguity,
        tags,
        distance,
        PoseObservationType.PHOTONVISION);
  }

  @Test
  void rejectsSingleTagAmbiguityAndRangeButAcceptsMultiTag() {
    assertTrue(Vision.shouldRejectPoseObservation(observation(2, 2, 0, 1, 0.31, 2)));
    assertTrue(Vision.shouldRejectPoseObservation(observation(2, 2, 0, 1, -1, 2)));
    assertTrue(Vision.shouldRejectPoseObservation(observation(2, 2, 0, 1, 0.1, 3.01)));
    assertFalse(Vision.shouldRejectPoseObservation(observation(2, 2, 0, 1, 0.3, 3)));
    assertFalse(Vision.shouldRejectPoseObservation(observation(2, 2, 0, 2, 0.9, 5)));
  }

  @Test
  void rejectsImpossiblePositionsAndMissingTags() {
    assertTrue(Vision.shouldRejectPoseObservation(observation(-0.01, 2, 0, 2, 0, 2)));
    assertTrue(Vision.shouldRejectPoseObservation(observation(2, -0.01, 0, 2, 0, 2)));
    assertTrue(
        Vision.shouldRejectPoseObservation(
            observation(VisionConstants.aprilTagLayout.getFieldLength() + 0.01, 2, 0, 2, 0, 2)));
    assertTrue(
        Vision.shouldRejectPoseObservation(
            observation(2, VisionConstants.aprilTagLayout.getFieldWidth() + 0.01, 0, 2, 0, 2)));
    assertTrue(Vision.shouldRejectPoseObservation(observation(2, 2, 0.76, 2, 0, 2)));
    assertTrue(Vision.shouldRejectPoseObservation(observation(2, 2, -0.76, 2, 0, 2)));
    assertTrue(Vision.shouldRejectPoseObservation(observation(2, 2, 0, 0, 0, 2)));
    assertFalse(Vision.shouldRejectPoseObservation(observation(0, 0, 0, 2, 0, 2)));
  }

  @Test
  void rejectsNonfiniteDataAndZeroDistance() {
    assertTrue(Vision.shouldRejectPoseObservation(observation(Double.NaN, 2, 0, 2, 0, 2)));
    assertTrue(Vision.shouldRejectPoseObservation(observation(2, 2, 0, 2, 0, Double.NaN)));
    assertTrue(Vision.shouldRejectPoseObservation(observation(2, 2, 0, 2, 0, 0)));
    assertTrue(Vision.shouldRejectPoseObservation(observation(2, 2, 0, 1, Double.NaN, 2)));
    assertTrue(
        Vision.shouldRejectPoseObservation(
            new PoseObservation(
                Double.NaN, new Pose3d(), 0.1, 2, 2, PoseObservationType.PHOTONVISION)));
  }

  @Test
  void fusesEveryAcceptedFrameWithDistanceWeightingAndSingleTagHeadingSuppression() {
    var first = observation(2, 2, 0, 2, 0, 2);
    var second = observation(3, 2, 0, 1, 0.1, 2);
    var rejected = observation(-1, 2, 0, 2, 0, 2);
    List<Measurement> received = new ArrayList<>();
    var io =
        new VisionIO() {
          private boolean delivered;

          @Override
          public void updateInputs(VisionIOInputs inputs) {
            inputs.connected = true;
            inputs.poseObservations =
                delivered
                    ? new PoseObservation[0]
                    : new PoseObservation[] {first, rejected, second};
            delivered = true;
          }
        };
    var vision =
        new Vision((pose, time, stdDevs) -> received.add(new Measurement(pose, time, stdDevs)), io);
    try {
      vision.periodic();
      assertTrue(vision.isConnected());
      assertEquals(2, received.size());
      assertEquals(first.pose().toPose2d(), received.get(0).pose());
      assertEquals(1.0, received.get(0).timestamp());
      assertEquals(0.04, received.get(0).stdDevs().get(0, 0), 1e-9);
      assertEquals(0.12, received.get(0).stdDevs().get(2, 0), 1e-9);
      assertEquals(0.08, received.get(1).stdDevs().get(1, 0), 1e-9);
      assertEquals(1000.0, received.get(1).stdDevs().get(2, 0));
      vision.periodic();
      assertEquals(2, received.size(), "No stale frame may be reapplied");
    } finally {
      CommandScheduler.getInstance().unregisterSubsystem(vision);
    }
  }

  @Test
  void recordedObservationBatchRoundTripsAndIsConsumedInTheSameCycle() {
    var recorded = new VisionIOInputsAutoLogged();
    recorded.connected = true;
    recorded.tagIds = new int[] {1, 2};
    recorded.poseObservations = new PoseObservation[] {observation(2, 2, 0, 2, 0, 2)};
    var table = new LogTable(0);
    recorded.toLog(table);
    var restored = new VisionIOInputsAutoLogged();
    restored.fromLog(table);
    assertArrayEquals(recorded.poseObservations, restored.poseObservations);
    assertArrayEquals(recorded.tagIds, restored.tagIds);
    List<Pose2d> received = new ArrayList<>();
    var vision =
        new Vision(
            (pose, time, stdDevs) -> received.add(pose),
            new VisionIO() {
              @Override
              public void updateInputs(VisionIOInputs inputs) {
                ((VisionIOInputsAutoLogged) inputs).fromLog(table);
              }
            });
    try {
      vision.periodic();
      assertEquals(List.of(recorded.poseObservations[0].pose().toPose2d()), received);
    } finally {
      CommandScheduler.getInstance().unregisterSubsystem(vision);
    }
  }

  private record Measurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {}
}
