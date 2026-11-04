package frc2713.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

class JetsonCameraSupportTest {
  private static PhotonPipelineResult frame(long sequence, int id) {
    var corners =
        List.of(
            new TargetCorner(0, 0),
            new TargetCorner(1, 0),
            new TargetCorner(1, 1),
            new TargetCorner(0, 1));
    var target =
        new PhotonTrackedTarget(
            0, 0, 1, 0, id, -1, 0, Transform3d.kZero, Transform3d.kZero, 0.1, corners, corners);
    return new PhotonPipelineResult(sequence, 1_000_000, 1_050_000, 0, List.of(target));
  }

  @Test
  void matchesByFrameAndTagWithoutMakingQualityMandatory() {
    try (var support = new JetsonCameraSupport("quality-match-test", Transform3d.kZero)) {
      support.offer(new double[] {7, 1, 2, 50, 60, 0.1, 0.2, 0.3});
      assertTrue(support.match(frame(6, 2)).isEmpty());
      var matched = support.match(frame(7, 2));
      assertEquals(1, matched.size());
      assertEquals(7, matched.get(0).sequenceId());
      assertEquals(2, matched.get(0).tagId());
      assertEquals(50, matched.get(0).decisionMargin());
      assertTrue(support.match(frame(7, 2)).isEmpty(), "Quality is consumed only once");
      support.offer(new double[] {8, 1, 2, 50, 60, 0.1, 0.2, 0.3});
      assertTrue(support.match(frame(8, 3)).isEmpty(), "Never attach another tag's quality");
      assertEquals(1, support.matchedFrames);
      assertEquals(3, support.unmatchedFrames);
    }
  }

  @Test
  void ignoresMalformedPacketsBoundsQueueAndClearsOnRestart() {
    try (var support = new JetsonCameraSupport("quality-invalid-test", Transform3d.kZero)) {
      for (var bad :
          List.of(
              new double[0],
              new double[] {1, Double.NaN},
              new double[] {Double.NaN, 0},
              new double[] {1, 1.5},
              new double[] {1, 1, 2})) support.offer(bad);
      assertTrue(support.match(frame(1, 2)).isEmpty());
      for (int sequence = 1; sequence <= 33; sequence++)
        support.offer(new double[] {sequence, 1, 2, 50, 60, 0.1, 0.2, 0.3});
      assertTrue(support.match(frame(1, 2)).isEmpty());
      support.offer(new double[] {0, 1, 2, 50, 60, 0.1, 0.2, 0.3});
      assertTrue(support.match(frame(33, 2)).isEmpty());
      assertEquals(1, support.match(frame(0, 2)).size());
    }
  }

  @Test
  void publishesMountAndEnableRequestForTheCameraLifetime() {
    var table = NetworkTableInstance.getDefault().getTable("photonvision/quality-mount-test");
    var mount = new Transform3d(0.3, -0.1, 0.5, new Rotation3d(0, -0.2, 0));
    try (var enabled = table.getBooleanTopic("enabledRequest").subscribe(false);
        var transform =
            table.getStructTopic("robotToCamera", Transform3d.struct).subscribe(Transform3d.kZero);
        var support = new JetsonCameraSupport("quality-mount-test", mount)) {
      assertTrue(enabled.get());
      assertEquals(mount, transform.get());
    }
  }
}
