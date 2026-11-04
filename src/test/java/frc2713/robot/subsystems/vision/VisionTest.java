package frc2713.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc2713.robot.Constants;
import frc2713.robot.subsystems.vision.VisionConstants.CameraConfig;
import frc2713.robot.subsystems.vision.VisionIO.PoseObservation;
import frc2713.robot.subsystems.vision.VisionIO.TagQuality;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.IntStream;
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
      double timestamp, double x, int tags, double ambiguity, double distance) {
    return new PoseObservation(
        timestamp, new Pose3d(x, 2, 0, Rotation3d.kZero), ambiguity, tags, distance);
  }

  @Test
  void rejectsAmbiguousOrDistantSingleTagsButAcceptsMultiTag() {
    assertEquals("Single-tag ambiguity", Vision.rejectionReason(observation(1, 2, 1, 0.31, 2), 1));
    assertEquals("Single-tag ambiguity", Vision.rejectionReason(observation(1, 2, 1, -1, 2), 1));
    assertEquals("Single-tag range", Vision.rejectionReason(observation(1, 2, 1, 0.1, 3.01), 1));
    assertEquals("", Vision.rejectionReason(observation(1, 2, 1, 0.3, 3), 1));
    assertEquals("", Vision.rejectionReason(observation(1, 2, 2, 0.9, 5), 1));
  }

  @Test
  void rejectsInvalidGeometryNumbersAndTimestamps() {
    assertEquals("Outside field", Vision.rejectionReason(observation(1, -0.01, 2, 0, 2), 1));
    assertEquals(
        "Outside field",
        Vision.rejectionReason(
            observation(1, VisionConstants.aprilTagLayout.getFieldLength() + 0.01, 2, 0, 2), 1));
    for (double y : new double[] {-0.01, VisionConstants.aprilTagLayout.getFieldWidth() + 0.01}) {
      assertEquals(
          "Outside field",
          Vision.rejectionReason(
              new PoseObservation(1, new Pose3d(2, y, 0, Rotation3d.kZero), 0, 2, 2), 1));
    }
    for (double z : new double[] {-0.76, 0.76}) {
      assertEquals(
          "Robot height",
          Vision.rejectionReason(
              new PoseObservation(1, new Pose3d(2, 2, z, Rotation3d.kZero), 0, 2, 2), 1));
    }
    assertEquals(
        "Robot tilt",
        Vision.rejectionReason(
            new PoseObservation(1, new Pose3d(2, 2, 0, new Rotation3d(0.4, 0, 0)), 0, 2, 2), 1));
    assertEquals(
        "Robot tilt",
        Vision.rejectionReason(
            new PoseObservation(1, new Pose3d(2, 2, 0, new Rotation3d(0, -0.4, 0)), 0, 2, 2), 1));
    assertEquals("No tags", Vision.rejectionReason(observation(1, 2, 0, 0, 2), 1));
    assertEquals(
        "Invalid measurement", Vision.rejectionReason(observation(Double.NaN, 2, 2, 0, 2), 1));
    assertEquals(
        "Invalid measurement", Vision.rejectionReason(observation(1, Double.NaN, 2, 0, 2), 1));
    assertEquals(
        "Invalid measurement",
        Vision.rejectionReason(observation(1, 2, 2, 0, Double.POSITIVE_INFINITY), 1));
    assertEquals("Invalid measurement", Vision.rejectionReason(observation(1, 2, 2, 0, 0), 1));
    assertEquals(
        "Single-tag ambiguity", Vision.rejectionReason(observation(1, 2, 1, Double.NaN, 2), 1));
    assertEquals("Stale timestamp", Vision.rejectionReason(observation(1, 2, 2, 0, 2), 1.51));
    assertEquals("Future timestamp", Vision.rejectionReason(observation(1.06, 2, 2, 0, 2), 1));
    assertEquals("", Vision.rejectionReason(observation(1, 0, 2, 0, 2), 1));
  }

  @Test
  void fusesEachCameraFrameOnceInTimestampOrderAndRejectsLateFrames() {
    var io0 = new TestIO(observation(1.04, 3, 1, 0.1, 2), observation(1.02, 2, 2, 0, 2));
    var io1 = new TestIO(observation(1.03, 4, 2, 0, 2), observation(1.04, 5, 2, 0, 2));
    List<Measurement> received = new ArrayList<>();
    try (var vision = createVision(received, new ArrayList<>(), new double[] {1.1}, io0, io1)) {
      vision.periodic();
      assertTrue(vision.isConnected());
      assertEquals(
          List.of(1.02, 1.03, 1.04, 1.04), received.stream().map(Measurement::timestamp).toList());
      assertEquals(0.04, received.get(0).stdDevs().get(0, 0), 1e-9);
      assertEquals(0.12, received.get(0).stdDevs().get(2, 0), 1e-9);
      assertEquals(0.08, received.get(2).stdDevs().get(0, 0), 1e-9);
      assertEquals(VisionConstants.ignoredAngularStdDev, received.get(2).stdDevs().get(2, 0));
      vision.periodic(); // The test IO repeats the same data intentionally.
      assertEquals(4, received.size());
      io0.observations = new PoseObservation[] {observation(1.01, 2, 2, 0, 2)};
      vision.periodic();
      assertEquals(4, received.size());
      // A bad future timestamp must not block later valid camera data.
      io0.observations =
          new PoseObservation[] {observation(100, 2, 2, 0, 2), observation(1.05, 2, 2, 0, 2)};
      vision.periodic();
      assertEquals(5, received.size());
    }
  }

  @Test
  void uncertaintyHasFloorsAndCameraScaling() {
    var std = Vision.standardDeviations(observation(1, 2, 2, 0, 0.1), 2);
    assertEquals(VisionConstants.minLinearStdDev * 2, std.get(0, 0));
    assertEquals(VisionConstants.minAngularStdDev * 2, std.get(2, 0));
  }

  @Test
  void resetRequiresFreshAcceptedPoseAllowsZeroXAndPreservesSingleTagHeading() {
    var io = new TestIO(observation(1, 0, 1, 0.1, 2));
    double[] now = {1.1};
    List<Pose2d> resets = new ArrayList<>();
    try (var vision = createVision(new ArrayList<>(), resets, now, io)) {
      vision.hardResetDrivePose();
      assertTrue(resets.isEmpty());
      vision.periodic();
      assertTrue(vision.getPose().isPresent());
      now[0] = 1.26;
      vision.hardResetDrivePose();
      assertTrue(resets.isEmpty(), "Stale accepted pose must not reset the robot");
      io.observations = new PoseObservation[] {observation(1.27, 0, 1, 0.1, 2)};
      now[0] = 1.3;
      vision.periodic();
      vision.hardResetDrivePose();
      assertEquals(List.of(new Pose2d(0, 2, Rotation2d.fromDegrees(90))), resets);
      vision.hardResetDrivePose();
      assertEquals(1, resets.size(), "Do not reuse a reset observation");
    }
  }

  @Test
  void recordedBatchRoundTripsAndFeedsTheCommonPipelineInTheSameCycle() {
    var recorded = new VisionIOInputsAutoLogged();
    recorded.connected = true;
    recorded.frameCount = 1;
    recorded.tagIds = new int[] {1, 2};
    recorded.poseObservations = new PoseObservation[] {observation(1, 2, 2, 0, 2)};
    recorded.tagQuality = new TagQuality[] {new TagQuality(7, 1, 1, 40, 60, 0.1, 0.2, Double.NaN)};
    var table = new LogTable(1_100_000);
    recorded.toLog(table);
    var restored = new VisionIOInputsAutoLogged();
    restored.fromLog(table);
    assertArrayEquals(recorded.poseObservations, restored.poseObservations);
    assertArrayEquals(recorded.tagQuality, restored.tagQuality);
    assertArrayEquals(recorded.tagIds, restored.tagIds);
    List<Measurement> received = new ArrayList<>();
    try (var vision =
        createVision(
            received,
            new ArrayList<>(),
            new double[] {1.1},
            new VisionIO() {
              @Override
              public void updateInputs(VisionIOInputs inputs) {
                ((VisionIOInputsAutoLogged) inputs).fromLog(table);
              }
            })) {
      vision.periodic();
      assertEquals(
          List.of(recorded.poseObservations[0].pose().toPose2d()),
          received.stream().map(Measurement::pose).toList());
      vision.periodic();
      assertEquals(1, received.size());
    }
  }

  private static Vision createVision(
      List<Measurement> received, List<Pose2d> resets, double[] now, VisionIO... io) {
    var configs =
        IntStream.range(0, io.length)
            .mapToObj(i -> new CameraConfig("test" + i, Transform3d.kZero, 1))
            .toList();
    return new Vision(
        (pose, time, std) -> received.add(new Measurement(pose, time, std)),
        () -> new Pose2d(2, 2, Rotation2d.fromDegrees(90)),
        resets::add,
        configs,
        () -> now[0],
        io);
  }

  private static class TestIO implements VisionIO {
    PoseObservation[] observations;

    TestIO(PoseObservation... observations) {
      this.observations = observations;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
      inputs.connected = true;
      inputs.poseObservations = observations;
      inputs.frameCount = observations.length;
    }
  }

  private record Measurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {}
}
