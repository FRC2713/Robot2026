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
import edu.wpi.first.networktables.StringArraySubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc2713.lib.io.AdvantageScopePathBuilder;
import frc2713.lib.util.LoggedTunableNumber;
import frc2713.robot.FieldConstants;
import frc2713.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;
import org.zeromq.SocketType;
import org.zeromq.ZContext;
import org.zeromq.ZMQ;

public class VisionIOSLAMDunk implements VisionIO {
  private NetworkTableInstance inst;
  private NetworkTable table;
  private DoubleArraySubscriber poseSub;
  private DoubleArraySubscriber scTagsSub;
  private StringArraySubscriber scErrorsSub;

  private final AdvantageScopePathBuilder pb = new AdvantageScopePathBuilder("Vision");

  private double lastTimestamp = -1;
  private static final Transform3d SLAMDUNK_TRANSFORM =
      new Transform3d(new Translation3d(), new Rotation3d(0, 0, Math.PI / 2));

  private final Time WARN_AFTER_NO_UPDATES_FOR = Seconds.of(4);
  private final Alert slamdunkAlert =
      new Alert(
          "Vision/Alerts",
          "No SLAMDunk! updates for >" + WARN_AFTER_NO_UPDATES_FOR,
          AlertType.kWarning);
  private final Alert gyroAlert =
      new Alert("Vision/Alerts", "Failed to send gyro update to SuperCap", AlertType.kError);
  private final Alert[] scAlerts = new Alert[10];

  private final ZContext supercapContext;
  private final ZMQ.Socket superCapSocket;

  private static final LoggedTunableNumber kDistance =
      new LoggedTunableNumber("Vision/kDistance", 3);

  private final LoggedTunableNumber starvationThreshold =
      new LoggedTunableNumber("Vision/starvationThresholdSec", 1.0);
  private final LoggedTunableNumber cStarvation =
      new LoggedTunableNumber("Vision/cStarvation", 1.5);
  private final LoggedTunableNumber nStarvation =
      new LoggedTunableNumber("Vision/nStarvation", 1); // cast to int

  public VisionIOSLAMDunk() {
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("slamdunk");
    poseSub = table.getDoubleArrayTopic("pose_robot").subscribe(new double[0]);
    scTagsSub = table.getDoubleArrayTopic("supercap_tags").subscribe(new double[0]);
    scErrorsSub = table.getStringArrayTopic("supercap_alerts/errors").subscribe(new String[0]);

    supercapContext = new ZContext();
    superCapSocket = supercapContext.createSocket(SocketType.REQ);
    superCapSocket.setSendTimeOut(500); // timeout for sending gyro updates
    superCapSocket.setReceiveTimeOut(500); // timeout for receiving replies
    superCapSocket.connect(VisionConstants.SUPERCAP_IPC_ADDRESS);

    for (int i = 0; i < scAlerts.length; i++) {
      scAlerts[i] = new Alert("Vision/Alerts", "", AlertType.kError);
    }
  }

  @Override
  public void updateInputs(VisionInputs inputs) {
    // inputs.translationStdDev = VisionConstants.POSE_ESTIMATOR_STATE_STDEVS.translationalStDev();
    // inputs.rotationStdDev = VisionConstants.POSE_ESTIMATOR_STATE_STDEVS.rotationalStDev();

    // Reset pose to zero, leave pose3d as last state for visualization
    inputs.pose = new Pose2d();

    if (Timer.getFPGATimestamp() - lastTimestamp > WARN_AFTER_NO_UPDATES_FOR.in(Seconds)) {
      slamdunkAlert.set(true);
      SmartDashboard.putBoolean("Vision/SLAMDunkUpdates", false);
    } else {
      slamdunkAlert.set(false);
      SmartDashboard.putBoolean("Vision/SLAMDunkUpdates", true);
    }

    var poseArray = poseSub.get();
    Logger.recordOutput(pb.makePath("SLAMDunk Array"), poseArray);

    var scArray = scTagsSub.get();
    Logger.recordOutput(pb.makePath("SuperCap Tags Array"), scArray);

    var scErrorsArray = scErrorsSub.get();
    Logger.recordOutput(pb.makePath("SuperCap Errors Array"), scErrorsArray);

    for (int i = 0; i < scAlerts.length; i++) {
      if (i < scErrorsArray.length) {
        scAlerts[i].setText(scErrorsArray[i]);
        scAlerts[i].set(true);
      } else {
        scAlerts[i].set(false);
      }
    }

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
            inputs.lastAppliedTimestamp = inputs.timestamp;
            RobotContainer.drive.setPose(inputs.pose);
            Logger.recordOutput(pb.makePath("robotOutsideField"), true);
            return;
          } else {
            Logger.recordOutput(pb.makePath("robotOutsideField"), false);
          }
        }

        // Logger.recordOutput(
        //     pb.makePath("distanceTag9"),
        //     FieldConstants.AprilTagLayoutType.OFFICIAL
        //         .getLayout()
        //         .getTagPose(9)
        //         .get()
        //         .getTranslation()
        //         .getDistance(inputs.pose3d.getTranslation()));

        if (!FieldConstants.FIELD.contains(inputs.pose.getTranslation())) {
          inputs.reasoning = "Vision outside field";
          inputs.applying = false;
          return;
        }

        if (Math.abs(inputs.pose3d.getTranslation().getZ()) > 0.1) {
          inputs.applying = false;
          inputs.reasoning = "Off ground Z > 0.1m";
          return;
        }

        double distanceScaleFactor = distanceScaleFactor(inputs);
        double countScaleFactor = countScaleFactor(inputs);
        double starvationScaleFactor = starvationScaleFactor(inputs, false);
        double finalScaleFactor = distanceScaleFactor * countScaleFactor * starvationScaleFactor;

        Logger.recordOutput(pb.makePath("distanceScaleFactor"), distanceScaleFactor);
        Logger.recordOutput(pb.makePath("countScaleFactor"), countScaleFactor);
        Logger.recordOutput(pb.makePath("starvationScaleFactor"), starvationScaleFactor);
        Logger.recordOutput(pb.makePath("finalScaleFactor"), finalScaleFactor);

        inputs.translationStdDev =
            VisionConstants.POSE_ESTIMATOR_STATE_STDEVS
                .translationalStDev()
                .times(finalScaleFactor);
        inputs.rotationStdDev = Degrees.of(99999);
        inputs.reasoning = "Valid pose.";
        inputs.applying = true;
        inputs.lastAppliedTimestamp = inputs.timestamp;
        return;
      }
    }

    if (poseArray.length <= 10) {
      inputs.reasoning = "Invalid pose data.";
    } else {
      inputs.reasoning = "Stale timestamp";
    }
    inputs.applying = false;
    return;
  }

  private double roughDist(VisionInputs inputs) {
    return 74.7 * Math.pow(inputs.avgTagSize, -0.396);
  }

  /** A penalty for average tag size being farther away. */
  private double distanceScaleFactor(VisionInputs inputs) {
    return Math.pow(roughDist(inputs), kDistance.get());
  }

  /** A bonus for seeing more tags. */
  private double countScaleFactor(VisionInputs inputs) {
    return 1 / Math.max(1, Math.pow(inputs.tagCount, 2));
  }

  /**
   * A bonus for being the first reported pose after a period of starvation.
   *
   * <p>If we were starved for vision poses, then odometry has had a lot of time to drift. So, when
   * we finally get a new vision pose, it's impact on the pose estimate is too low to correct drift
   * for a while.
   */
  private double starvationScaleFactor(VisionInputs inputs, boolean autoOnly) {
    double timeElapsed = inputs.timestamp - inputs.lastAppliedTimestamp;
    if (timeElapsed < starvationThreshold.get() || (autoOnly && !DriverStation.isAutonomous())) {
      return 1.0; // Not starved yet
    }

    double excessTime = timeElapsed - starvationThreshold.get();
    return 1 / (cStarvation.get() * Math.pow(1 + excessTime, (int) nStarvation.get()));
  }

  @Override
  public void setGyroAngle(Angle angle) {
    new Thread(
            () -> {
              try {

                superCapSocket.sendMore("reset_gyro");
                boolean success =
                    superCapSocket.send(
                        String.valueOf(
                            angle
                                .minus(SLAMDUNK_TRANSFORM.getRotation().getMeasureZ())
                                .in(Degrees)));

                if (success) {
                  byte[] reply = superCapSocket.recv(0);
                  String replyStr = reply != null ? new String(reply) : "null";
                  System.out.println("Received reply from SuperCap: " + replyStr);
                  if (reply == null || !replyStr.equals("OK")) {
                    gyroAlert.set(true);
                    System.err.println("Unexpected reply from SuperCap: " + replyStr);
                    Logger.recordOutput(pb.makePath("SuperCapReplyError"), replyStr);
                  } else {
                    gyroAlert.set(false);
                  }
                } else {
                  gyroAlert.set(true);
                }
              } catch (Exception e) {
                gyroAlert.set(true);
                System.err.println("Error sending gyro update to SuperCap: " + e.getMessage());
                Logger.recordOutput(pb.makePath("ZmqError"), e.getMessage());
              }
            })
        .start();
  }
}
