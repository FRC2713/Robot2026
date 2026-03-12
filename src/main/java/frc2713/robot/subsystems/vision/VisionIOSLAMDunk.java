package frc2713.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc2713.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class VisionIOSLAMDunk implements VisionIO {
  private NetworkTableInstance inst;
  private NetworkTable table;
  private DoubleArraySubscriber sub;
  private double lastTimestamp = -1;

  public VisionIOSLAMDunk() {
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("slamdunk");
    sub = table.getDoubleArrayTopic("pose_robot").subscribe(new double[0]);
  }

  @Override
  public void updateInputs(VisionInputs inputs) {
    inputs.translationStdDev = VisionConstants.POSE_ESTIMATOR_STATE_STDEVS.translationalStDev();
    inputs.rotationStdDev = VisionConstants.POSE_ESTIMATOR_STATE_STDEVS.rotationalStDev();

    // Reset pose to zero, leave pose3d as last state for visualization
    inputs.pose = new Pose2d();

    var linspeed = RobotContainer.drive.getSpeed();
    var angspeed = RobotContainer.drive.getAngularSpeed();
    Logger.recordOutput("SLAMDunk/SpeedLinear", linspeed);
    Logger.recordOutput("SLAMDunk/SpeedAngular", angspeed);
    Logger.recordOutput("SLAMDunk/tFPGA", Timer.getFPGATimestamp());

    var poseArray = sub.get();
    if (poseArray.length > 0) {
      double t = poseArray[0];

      if (lastTimestamp != t && poseArray.length > 7) {

        inputs.timestamp = t;
        double latency = Timer.getFPGATimestamp() - t;
        inputs.latency = Seconds.of(latency);

        lastTimestamp = t;
        inputs.pose3d =
            new Pose3d(
                new Translation3d(poseArray[1], poseArray[2], poseArray[3]),
                new Rotation3d(
                    new Quaternion(poseArray[4], poseArray[5], poseArray[6], poseArray[7])));

        inputs.pose = inputs.pose3d.toPose2d();

        // pose =
        //     pose.transformBy(
        //         new Transform3d(new Translation3d(), new Rotation3d(0, 0, Math.PI / 2)));

        if (latency < 0) {
          inputs.reasoning = "Pose from the future!";
          inputs.applying = false;
          return;
        }
        if (inputs.latency.compareTo(VisionConstants.LATENCY_THRESHOLD.get()) > 0) {
          inputs.reasoning = "Latency above threshold";
          inputs.applying = false;
          return;
        }

        if ((inputs
                .pose
                .getTranslation()
                .getDistance(RobotContainer.drive.getPose().getTranslation())
            > VisionConstants.MAX_POSE_JUMP.in(Meters))) {
          inputs.reasoning = "Jump protection";
          inputs.applying = false;
          return;
        }

        if (Math.abs(inputs.pose3d.getTranslation().getZ()) > 0.1) {
          inputs.applying = false;
          inputs.reasoning = "Off ground Z > 0.1m";
          return;
        }

        inputs.applying = true;
        inputs.reasoning = "Valid pose";

        return;
      }
    }

    if (poseArray.length <= 7) {
      inputs.reasoning = "No pose data available";
    } else {
      inputs.reasoning = "Stale timestamp";
    }
    inputs.applying = false;
    return;
  }
}
