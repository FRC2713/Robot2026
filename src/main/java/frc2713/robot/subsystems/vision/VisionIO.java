package frc2713.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

  @AutoLog
  public static class VisionInputs {
    public Pose2d pose = new Pose2d();
    public Pose3d pose3d = new Pose3d();
    public double timestamp;

    public int tagCount;

    public boolean applying;
    public String reasoning;

    public double translationStdDev =
        VisionConstants.POSE_ESTIMATOR_STATE_STDEVS.translationalStDev();
    public double rotationStdDev = VisionConstants.POSE_ESTIMATOR_STATE_STDEVS.rotationalStDev();
  }

  public default void update(VisionInputs inputs) {}
}
