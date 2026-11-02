package frc2713.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc2713.robot.FieldConstants.AprilTagLayoutType;
import frc2713.robot.RobotContainer;
import java.util.LinkedHashMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.SimCameraProperties;
import org.spectrum3847.vision.VisionSystem;
import org.spectrum3847.vision.VisionUpdate;
import org.spectrum3847.vision.gate.GateSet;
import org.spectrum3847.vision.log.VisionLogger;
import org.spectrum3847.vision.sim.VisionSim;

/** Also drives Rewind: records the cameras from auto start until teleop ends. */
public class VisionIOSpectrum implements VisionIO {
  private static final double REWIND_DISABLED_TIMEOUT_SEC = 10.0;
  private static final Pose2d[] NO_POSES = new Pose2d[0];

  private final VisionSystem vision;
  private final VisionSim simulation;
  private final Supplier<Pose2d> simulationPose;

  private final BooleanPublisher rewindRecord =
      NetworkTableInstance.getDefault()
          .getTable("photonvision/rewind")
          .getBooleanTopic("record")
          .publish();
  private boolean recording = false;
  private boolean teleopRan = false;
  private double lastEnabledTimestamp = 0;

  public VisionIOSpectrum() {
    this(null);
  }

  /**
   * Creates simulated cameras when a true simulation pose supplier is provided. The no-argument
   * constructor uses physical cameras, including for bench testing with a connected Jetson.
   */
  public VisionIOSpectrum(Supplier<Pose2d> simulationPose) {
    this.simulationPose = simulationPose;
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
    // Recommended gates for competition:
    // var gates = GateSet.of(
    //     Gates.uniqueFrames(),
    //     Gates.timeSynced(0.5),
    //     Gates.maxAge(0.5),
    //     Gates.notFromFuture(0.05),
    //     Gates.excludedTags(),
    //     Gates.insideField(0.5),
    //     Gates.maxHeight(0.3),
    //     Gates.maxTilt(20),
    //     Gates.maxSingleTagAmbiguity(0.3),
    //     Gates.maxDistance(4.5, 7.0),
    //     Gates.maxReprojection(3.0),
    //     Gates.maxYawRate(3.5),
    //     Gates.headingAgreesWithGyro(15),
    //     Gates.innovation(new Gates.InnovationConfig()));
    vision =
        builder
            .gates(gates)
            .sink((pose, timestamp, stdDevs) -> {})
            .currentEstimate(() -> RobotContainer.drive.getPose())
            .logger(VisionLogger.NONE)
            .build();
    if (simulationPose != null) {
      // Match the PhotonVision branch's camera model. VisionSim attaches every configured camera
      // to the official field layout and disables video streams and wireframe rendering.
      var properties = new SimCameraProperties();
      properties.setCalibration(1600, 1200, Rotation2d.fromDegrees(90));
      properties.setCalibError(0.4, 0.10);
      properties.setFPS(25);
      properties.setAvgLatencyMs(50);
      properties.setLatencyStdDevMs(15);
      simulation = VisionSim.attach(vision, properties);
      simulation
          .visionSystemSim()
          .getCameraSims()
          .forEach(camera -> camera.setMaxSightRange(Units.feetToMeters(22.0)));
    } else {
      simulation = null;
    }
    rewindRecord.set(false);
  }

  @Override
  public void updateInputs(VisionInputs inputs) {
    if (simulation != null) {
      // Generate frames before consuming them, using wheel/gyro truth rather than the estimate.
      simulation.update(simulationPose.get());
    } else if (VisionConstants.REWIND_ENABLED) {
      updateRewind();
    }

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
    logVisualization(update);
  }

  /** Logs field-relative tag targets for AdvantageScope's Vision Target display. */
  private void logVisualization(VisionUpdate update) {
    var field = AprilTagLayoutType.OFFICIAL.getLayout();
    var robotPose = new Pose3d(RobotContainer.drive.getPose());
    var allTagPoses = new LinkedHashMap<Integer, Pose3d>();

    VisionConstants.CAMERA_POSES.forEach(
        (name, robotToCamera) -> {
          String path = "Vision/" + name;
          // The configured mount pose is robot-relative; CameraPose is field-relative so it
          // can be displayed as axes alongside the robot, even when no tags are visible.
          Logger.recordOutput(path + "/RobotToCameraPose", new Pose3d().transformBy(robotToCamera));
          Logger.recordOutput(path + "/CameraPose", robotPose.transformBy(robotToCamera));

          var tagPoses = new LinkedHashMap<Integer, Pose3d>();
          for (var frame : update.frames()) {
            if (!frame.camera().equals(name)) continue;
            for (var tag : frame.tags()) {
              field.getTagPose(tag.id()).ifPresent(pose -> tagPoses.put(tag.id(), pose));
            }
          }
          // Include detections even when their robot pose was rejected. Publish empty arrays
          // on loops without detections so stale sightlines are cleared.
          Logger.recordOutput(path + "/TagPoses", tagPoses.values().toArray(new Pose3d[0]));
          allTagPoses.putAll(tagPoses);
        });

    // Same summary key as the PhotonVision branch for existing AdvantageScope layouts.
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.values().toArray(new Pose3d[0]));
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
