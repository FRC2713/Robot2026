package frc2713.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

  @AutoLog
  public static class VisionInputs {
    // pose should be reset to zero if no new pose is available
    public Pose2d pose = new Pose2d();
    // pose3d can be left unchanged for visualization
    public Pose3d pose3d = new Pose3d();
    public double timestamp;
    public Time latency;

    public int tagCount;

    public boolean applying;
    public String reasoning;

    public Distance translationStdDev =
        VisionConstants.POSE_ESTIMATOR_STATE_STDEVS.translationalStDev();
    public Angle rotationStdDev = VisionConstants.POSE_ESTIMATOR_STATE_STDEVS.rotationalStDev();
  }

  public default void updateInputs(VisionInputs inputs) {}
}
