package frc2713.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc2713.robot.FieldConstants.AprilTagLayoutType;
import frc2713.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;
import org.spectrum3847.vision.VisionSystem;
import org.spectrum3847.vision.VisionUpdate;
import org.spectrum3847.vision.gate.GateSet;
import org.spectrum3847.vision.log.VisionLogger;

/** Also drives Rewind: records the cameras from auto start until teleop ends. */
public class VisionIOSpectrum implements VisionIO {
  private static final double REWIND_DISABLED_TIMEOUT_SEC = 10.0;
  private static final Pose2d[] NO_POSES = new Pose2d[0];

  private final VisionSystem vision;

  private final BooleanPublisher rewindRecord =
      NetworkTableInstance.getDefault()
          .getTable("photonvision/rewind")
          .getBooleanTopic("record")
          .publish();
  private boolean recording = false;
  private boolean teleopRan = false;
  private double lastEnabledTimestamp = 0;

  public VisionIOSpectrum() {
    var builder = VisionSystem.builder(AprilTagLayoutType.OFFICIAL.getLayout());
    VisionConstants.CAMERA_POSES.forEach(builder::camera);

    // Publish camera transforms to NT for PhotonVision to use
    var nt = NetworkTableInstance.getDefault();
    VisionConstants.CAMERA_POSES.forEach(
        (name, transform) -> {
          StructPublisher<Transform3d> pub =
              nt.getTable("photonvision")
                  .getSubTable(name)
                  .getStructTopic("robotToCamera", Transform3d.struct)
                  .publish();
          pub.set(transform);
        });

    // No gates for initial benchtop testing - accept everything to debug
    var gates = GateSet.of();
    vision =
        builder
            .gates(gates)
            .sink((pose, timestamp, stdDevs) -> {})
            .currentEstimate(() -> RobotContainer.drive.getPose())
            .logger(VisionLogger.NONE)
            .build();
    rewindRecord.set(false);
  }

  @Override
  public void updateInputs(VisionInputs inputs) {
    if (VisionConstants.REWIND_ENABLED) updateRewind();

    var speeds = RobotContainer.drive.getChassisSpeeds();
    vision.addMotion(
        Timer.getTimestamp(),
        RobotContainer.drive.getRotation(),
        speeds.omegaRadiansPerSecond,
        Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
    var update = vision.periodic();

    var latest = update.latestAccepted();
    if (latest != null) {
      var obs = latest.observation();
      inputs.pose = latest.pose();
      inputs.pose3d = obs.robotPose();
      inputs.rawYaw = inputs.pose.getRotation();

      inputs.timestamp = obs.timestampSeconds();
      inputs.latency = Seconds.of(update.nowSeconds() - obs.timestampSeconds());

      inputs.tagCount = obs.tagCount();
      inputs.translationStdDev = Meters.of(latest.stdDevs().get(0, 0));
      inputs.rotationStdDev = Radians.of(latest.stdDevs().get(2, 0));

      inputs.applying = true;
      inputs.lastAppliedTimestamp = inputs.timestamp;
      inputs.reasoning = obs.camera() + ": " + obs.type();

      applyAllAccepted(update);
    } else {
      inputs.applying = false;
      var rejected = update.rejected();
      if (!rejected.isEmpty()) {
        var r = rejected.get(rejected.size() - 1);
        inputs.reasoning = r.observation().camera() + ": " + r.gate() + ": " + r.reason();
      } else if (update.frames().isEmpty()) {
        inputs.reasoning = "No frames";
      }
    }

    // Debug logging
    Logger.recordOutput("Vision/RejectedPoses", update.rejectedPoses().toArray(NO_POSES));
    Logger.recordOutput("Vision/AcceptedPoses", update.acceptedPoses().toArray(NO_POSES));
    Logger.recordOutput("Vision/RewindRecording", recording);
    Logger.recordOutput("Vision/AcceptedCount", update.accepted().size());
    Logger.recordOutput("Vision/RejectedCount", update.rejected().size());
    Logger.recordOutput("Vision/FrameCount", update.frames().size());

    // Log raw camera inputs to see what PhotonVision is sending
    for (var cam : vision.cameras()) {
      var camInputs = cam.inputs();
      Logger.recordOutput("Vision/" + cam.name() + "/connected", camInputs.connected);
      Logger.recordOutput("Vision/" + cam.name() + "/resultCount", camInputs.resultCount);
      Logger.recordOutput("Vision/" + cam.name() + "/frameCount", camInputs.frames.size());
      // Log each frame's raw data
      for (int i = 0; i < camInputs.frames.size(); i++) {
        var frame = camInputs.frames.get(i);
        Logger.recordOutput(
            "Vision/" + cam.name() + "/frame" + i + "/tagCount", frame.tags().size());
        Logger.recordOutput("Vision/" + cam.name() + "/frame" + i + "/seqId", frame.sequenceId());
        if (frame.multiTagFieldToCamera() != null) {
          Logger.recordOutput(
              "Vision/" + cam.name() + "/frame" + i + "/multiTagFieldToCamera",
              frame.multiTagFieldToCamera());
        }
        // Log first tag's raw camera-to-tag transform (this should change when you move camera!)
        if (!frame.tags().isEmpty()) {
          var tag = frame.tags().get(0);
          Logger.recordOutput(
              "Vision/" + cam.name() + "/frame" + i + "/tag" + tag.id() + "/camToTag",
              tag.bestCameraToTag());
        }
      }
    }

    // Log the robotToCamera transform we're using
    for (var cam : vision.cameras()) {
      Logger.recordOutput("Vision/" + cam.name() + "/robotToCamera", cam.robotToCamera());
    }
  }

  /**
   * Applies all accepted measurements directly to drive. Called when there's at least one accepted.
   * This bypasses the single-pose Vision.java path to get all measurements in.
   */
  private void applyAllAccepted(VisionUpdate update) {
    var accepted = update.accepted();
    for (var a : accepted) {
      RobotContainer.drive.addVisionMeasurement(
          a.pose(), a.observation().timestampSeconds(), a.stdDevs());
    }
  }

  /**
   * Starts recording when auto enables, keeps going through the auto-to-teleop gap, and stops when
   * teleop ends (or after being disabled a while, for an auto-only run).
   */
  private void updateRewind() {
    boolean enabled = DriverStation.isEnabled();
    double now = Timer.getTimestamp();
    if (!recording) {
      if (enabled && DriverStation.isAutonomous()) {
        recording = true;
        teleopRan = false;
        lastEnabledTimestamp = now;
        rewindRecord.set(true);
      }
    } else if (enabled) {
      // dont call rewindRecord.set(true) over and over again
      lastEnabledTimestamp = now;
      teleopRan |= DriverStation.isTeleop();
    } else if (teleopRan || now - lastEnabledTimestamp > REWIND_DISABLED_TIMEOUT_SEC) {
      recording = false;
      rewindRecord.set(false);
    }
  }
}
