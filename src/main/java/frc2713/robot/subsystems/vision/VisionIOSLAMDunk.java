package frc2713.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc2713.lib.util.CachedPow;
import frc2713.lib.util.LoggedTunableNumber;
import frc2713.lib.util.MathUtil;
import frc2713.robot.FieldConstants;
import frc2713.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class VisionIOSLAMDunk implements VisionIO {
  private NetworkTableInstance inst;
  private NetworkTable table;
  private DoubleArraySubscriber sub;
  private double lastTimestamp = -1;
  private static final LoggedTunableNumber k =
      new LoggedTunableNumber("Vision/k", 2); // gets cast to int
  private static final Transform3d SLAMDUNK_TRANSFORM =
      new Transform3d(new Translation3d(), new Rotation3d(0, 0, Math.PI / 2));

  private CachedPow powN396 = new CachedPow(-0.396);

  public VisionIOSLAMDunk() {
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("slamdunk");
    sub = table.getDoubleArrayTopic("pose_robot").subscribe(new double[0]);
  }

  @Override
  public void updateInputs(VisionInputs inputs) {
    // inputs.translationStdDev = VisionConstants.POSE_ESTIMATOR_STATE_STDEVS.translationalStDev();
    // inputs.rotationStdDev = VisionConstants.POSE_ESTIMATOR_STATE_STDEVS.rotationalStDev();

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

      if (lastTimestamp != t && poseArray.length > 10) {
        inputs.tagCount = (int) poseArray[8];
        inputs.subgraphError = poseArray[9];
        inputs.avgTagSize = poseArray[10];
        inputs.timestamp = t;
        double latency = Timer.getFPGATimestamp() - t;
        inputs.latency = Seconds.of(latency);

        lastTimestamp = t;
        var rawPose =
            new Pose3d(
                new Translation3d(poseArray[1], poseArray[2], poseArray[3]),
                new Rotation3d(
                    new Quaternion(poseArray[4], poseArray[5], poseArray[6], poseArray[7])));

        inputs.rawYaw = Rotation2d.fromRadians(rawPose.getRotation().getZ());

        inputs.pose3d = rawPose.transformBy(SLAMDUNK_TRANSFORM);

        inputs.pose = inputs.pose3d.toPose2d();

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

        if (RobotContainer.drive != null) {
          if (!FieldConstants.FIELD_PLUS_HALF_METER.contains(
              RobotContainer.drive.getPose().getTranslation())) {
            inputs.reasoning = "ROBOT OUTSIDE FIELD!! HARD RESET";
            inputs.applying = true;
            RobotContainer.drive.setPose(inputs.pose);
            Logger.recordOutput("Vision/robotOutsideField", true);
            return;
          } else {
            Logger.recordOutput("Vision/robotOutsideField", false);
          }
        }

        Logger.recordOutput(
            "Vision/distanceTag9",
            FieldConstants.AprilTagLayoutType.OFFICIAL
                .getLayout()
                .getTagPose(9)
                .get()
                .getTranslation()
                .getDistance(inputs.pose3d.getTranslation()));

        if (!FieldConstants.FIELD_PLUS_HALF_METER.contains(inputs.pose.getTranslation())) {
          inputs.reasoning = "Vision outside field";
          inputs.applying = false;
          return;
        }

        if (Math.abs(inputs.pose3d.getTranslation().getZ()) > 0.1) {
          inputs.applying = false;
          inputs.reasoning = "Off ground Z > 0.1m";
          return;
        }

        // double poseDelta =
        //     inputs
        //         .pose
        //         .getTranslation()
        //         .getDistance(RobotContainer.drive.getPose().getTranslation());
        double roughDist = 74.7 * powN396.get(inputs.avgTagSize);
        Logger.recordOutput("Vision/roughDist", roughDist);
        double distScaleFactor = MathUtil.powDouble(roughDist, (int) k.get());
        Logger.recordOutput("Vision/distanceScaleFactor", distScaleFactor);
        double countScaleFactor = 1 / Math.max(1, MathUtil.sq(inputs.tagCount));
        Logger.recordOutput("Vision/countScaleFactor", countScaleFactor);

        inputs.translationStdDev =
            VisionConstants.POSE_ESTIMATOR_STATE_STDEVS
                .translationalStDev()
                .times(distScaleFactor)
                .times(countScaleFactor);
        inputs.rotationStdDev = Degrees.of(999);
        // VisionConstants.POSE_ESTIMATOR_STATE_STDEVS
        //     .rotationalStDev()
        //     .times(distScaleFactor)
        //     .times(countScaleFactor);

        inputs.reasoning = "Valid pose.";
        inputs.applying = true;
        return;
      }
    }

    if (poseArray.length <= 10) {
      inputs.reasoning = "No pose data available";
    } else {
      inputs.reasoning = "Stale timestamp";
    }
    inputs.applying = false;
    return;
  }
}
