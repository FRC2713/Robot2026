package frc2713.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructPublisher;
import frc2713.robot.subsystems.vision.VisionIO.TagQuality;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import org.photonvision.targeting.PhotonPipelineResult;

/** Optional SpectrumJetson extensions; stock PhotonVision can ignore all of these topics. */
final class JetsonCameraSupport implements AutoCloseable {
  private final BooleanPublisher enabled;
  private final StructPublisher<Transform3d> transform;
  private final DoubleArraySubscriber quality;
  private final LinkedHashMap<Long, double[]> pending = new LinkedHashMap<>();
  private long lastSequence = -1;
  long matchedFrames;
  long unmatchedFrames;

  JetsonCameraSupport(String name, Transform3d robotToCamera) {
    var table = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable(name);
    enabled = table.getBooleanTopic("enabledRequest").publish();
    transform = table.getStructTopic("robotToCamera", Transform3d.struct).publish();
    quality =
        table
            .getDoubleArrayTopic("tagQuality")
            .subscribe(
                new double[0],
                PubSubOption.sendAll(true),
                PubSubOption.keepDuplicates(true),
                PubSubOption.pollStorage(32));
    enabled.set(true);
    transform.set(robotToCamera);
  }

  void poll() {
    for (var value : quality.readQueue()) offer(value.value);
  }

  /**
   * Wire format: sequence, count, then id/margin/edge/undistortion/best error/alt error per tag.
   */
  void offer(double[] value) {
    if (value.length < 2
        || !Double.isFinite(value[0])
        || value[0] < 0
        || value[0] != Math.rint(value[0])
        || !Double.isFinite(value[1])
        || value[1] < 0
        || value[1] != Math.rint(value[1])
        || value[1] > (value.length - 2) / 6
        || value.length != 2 + 6 * (int) value[1]) return;
    long sequence = (long) value[0];
    if (sequence < lastSequence) pending.clear(); // Coprocessor restarted.
    lastSequence = sequence;
    pending.put(sequence, value.clone());
    if (pending.size() > 32) pending.remove(pending.keySet().iterator().next());
  }

  List<TagQuality> match(PhotonPipelineResult result) {
    var values = pending.remove(result.metadata.sequenceID);
    var matched = new ArrayList<TagQuality>();
    if (values != null) {
      for (var target : result.getTargets()) {
        for (int i = 2; i < values.length; i += 6) {
          if (values[i] == target.getFiducialId()) {
            matched.add(
                new TagQuality(
                    result.metadata.sequenceID,
                    result.getTimestampSeconds(),
                    target.getFiducialId(),
                    values[i + 1],
                    values[i + 2],
                    values[i + 3],
                    values[i + 4],
                    values[i + 5]));
            break;
          }
        }
      }
    }
    if (result.hasTargets()) {
      if (matched.isEmpty()) unmatchedFrames++;
      else matchedFrames++;
    }
    return matched;
  }

  @Override
  public void close() {
    enabled.close();
    transform.close();
    quality.close();
  }
}
