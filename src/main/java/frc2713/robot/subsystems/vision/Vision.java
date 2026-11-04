// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc2713.robot.subsystems.vision;

import static frc2713.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2713.lib.logging.PeriodicTimingLogger;
import frc2713.lib.logging.TimeLogged;
import frc2713.robot.Constants;
import frc2713.robot.subsystems.vision.VisionIO.PoseObservation;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/** One owner of vision validation, uncertainty, and fusion, shared by real, sim, and replay. */
public class Vision extends SubsystemBase implements AutoCloseable {
  private final VisionConsumer consumer;
  private final Supplier<Pose2d> robotPose;
  private final Consumer<Pose2d> resetPose;
  private final DoubleSupplier clock;
  private final CameraState[] cameras;
  private final RewindRecorder rewind;
  private final LoggedNetworkBoolean disableVision =
      new LoggedNetworkBoolean("Vision/disable", false);
  private PoseObservation latestAccepted;
  private double lastAppliedTimestamp = Double.NEGATIVE_INFINITY;

  public Vision(
      VisionConsumer consumer,
      Supplier<Pose2d> robotPose,
      Consumer<Pose2d> resetPose,
      List<CameraConfig> configs,
      VisionIO... io) {
    this(consumer, robotPose, resetPose, configs, Timer::getTimestamp, io);
  }

  Vision(
      VisionConsumer consumer,
      Supplier<Pose2d> robotPose,
      Consumer<Pose2d> resetPose,
      List<CameraConfig> configs,
      DoubleSupplier clock,
      VisionIO... io) {
    if (configs.size() != io.length
        || configs.stream().map(CameraConfig::name).distinct().count() != io.length) {
      throw new IllegalArgumentException("Each camera requires one unique configuration and IO");
    }
    this.consumer = consumer;
    this.robotPose = robotPose;
    this.resetPose = resetPose;
    this.clock = clock;
    cameras = new CameraState[io.length];
    for (int i = 0; i < io.length; i++) cameras[i] = new CameraState(configs.get(i), io[i]);
    rewind = new RewindRecorder(Constants.currentMode == Constants.Mode.REAL && REWIND_ENABLED);
  }

  @Override
  @TimeLogged("Performance/SubsystemPeriodic/Vision")
  public void periodic() {
    try (var ignored = PeriodicTimingLogger.time(this)) {
      rewind.periodic();
      double now = clock.getAsDouble();
      boolean enabled = !disableVision.get();
      var candidates = new ArrayList<Candidate>();
      var allTags = new ArrayList<Pose3d>();
      var seenTags = new HashSet<Integer>();
      for (var camera : cameras) {
        camera.clear();
        camera.io.updateInputs(camera.inputs);
        // Replay replaces inputs here, before any decision or estimator update.
        Logger.processInputs(camera.path, camera.inputs);
        camera.disconnected.set(!camera.inputs.connected);
        for (int id : camera.inputs.tagIds) {
          aprilTagLayout
              .getTagPose(id)
              .ifPresent(
                  pose -> {
                    camera.tags.add(pose);
                    if (seenTags.add(id)) allTags.add(pose);
                  });
        }
        for (var observation : camera.inputs.poseObservations)
          candidates.add(new Candidate(camera, observation));
      }
      candidates.sort(Comparator.comparingDouble(c -> c.observation.timestamp()));
      for (var candidate : candidates) {
        var camera = candidate.camera;
        var observation = candidate.observation;
        camera.poses.add(observation.pose());
        camera.ages.add(now - observation.timestamp());
        String reason = rejectionReason(observation, now);
        if (reason.isEmpty() && observation.timestamp() <= camera.lastTimestamp)
          reason = "Duplicate or out of order";
        if (reason.isEmpty() && observation.timestamp() < lastAppliedTimestamp)
          reason = "Older than last applied measurement";
        if (!reason.isEmpty()) {
          camera.rejected.add(observation.pose());
          camera.reasons.add(reason);
          continue;
        }
        camera.lastTimestamp = observation.timestamp();
        camera.accepted.add(observation.pose());
        if (latestAccepted == null || observation.timestamp() >= latestAccepted.timestamp())
          latestAccepted = observation;
        if (enabled) {
          var stdDevs = standardDeviations(observation, camera.config.stdDevFactor());
          consumer.accept(observation.pose().toPose2d(), observation.timestamp(), stdDevs);
          lastAppliedTimestamp = observation.timestamp();
          camera.applied.add(observation.pose());
          for (int axis = 0; axis < 3; axis++) camera.appliedStdDevs.add(stdDevs.get(axis, 0));
          camera.reasons.add("Applied");
        } else {
          camera.reasons.add("Fusion disabled");
        }
      }
      var pose = new Pose3d(robotPose.get());
      for (var camera : cameras) camera.log(pose);
      Logger.recordOutput("Vision/Summary/TagPoses", allTags.toArray(Pose3d[]::new));
      Logger.recordOutput(
          "Vision/Summary/RobotPoses",
          candidates.stream().map(c -> c.observation.pose()).toArray(Pose3d[]::new));
      Logger.recordOutput(
          "Vision/Summary/RobotPosesAccepted",
          Arrays.stream(cameras).flatMap(c -> c.accepted.stream()).toArray(Pose3d[]::new));
      Logger.recordOutput(
          "Vision/Summary/RobotPosesRejected",
          Arrays.stream(cameras).flatMap(c -> c.rejected.stream()).toArray(Pose3d[]::new));
      Logger.recordOutput(
          "Vision/Summary/RobotPosesApplied",
          Arrays.stream(cameras).flatMap(c -> c.applied.stream()).toArray(Pose3d[]::new));
      Logger.recordOutput("Vision/Connected", isConnected());
      Logger.recordOutput(
          "Vision/AnyConnected", Arrays.stream(cameras).anyMatch(c -> c.inputs.connected));
      Logger.recordOutput("Vision/Enabled", enabled);
      Logger.recordOutput(
          "Vision/Applying", Arrays.stream(cameras).anyMatch(c -> !c.applied.isEmpty()));
      Logger.recordOutput(
          "Vision/AcceptedCount", Arrays.stream(cameras).mapToInt(c -> c.accepted.size()).sum());
      Logger.recordOutput(
          "Vision/RejectedCount", Arrays.stream(cameras).mapToInt(c -> c.rejected.size()).sum());
      Logger.recordOutput(
          "Vision/AppliedCount", Arrays.stream(cameras).mapToInt(c -> c.applied.size()).sum());
      Logger.recordOutput(
          "Vision/FrameCount", Arrays.stream(cameras).mapToInt(c -> c.inputs.frameCount).sum());
      Logger.recordOutput(
          "Vision/LatestAcceptedAgeSeconds",
          latestAccepted == null ? -1 : now - latestAccepted.timestamp());
    }
  }

  /** Empty means accepted. Unknown single-tag ambiguity is rejected conservatively. */
  static String rejectionReason(PoseObservation observation, double now) {
    var pose = observation.pose();
    if (!Double.isFinite(observation.timestamp())
        || observation.timestamp() < 0
        || !Double.isFinite(pose.getX())
        || !Double.isFinite(pose.getY())
        || !Double.isFinite(pose.getZ())
        || !Double.isFinite(pose.getRotation().getX())
        || !Double.isFinite(pose.getRotation().getY())
        || !Double.isFinite(pose.getRotation().getZ())
        || !Double.isFinite(observation.averageTagDistance())
        || observation.averageTagDistance() <= 0) return "Invalid measurement";
    if (now - observation.timestamp() > maxMeasurementAgeSeconds) return "Stale timestamp";
    if (observation.timestamp() - now > futureTimestampToleranceSeconds) return "Future timestamp";
    if (observation.tagCount() <= 0) return "No tags";
    if (observation.tagCount() == 1) {
      if (!Double.isFinite(observation.ambiguity())
          || observation.ambiguity() < 0
          || observation.ambiguity() > maxAmbiguity) return "Single-tag ambiguity";
      if (observation.averageTagDistance() > maxSingleTagDistanceMeters) return "Single-tag range";
    }
    if (Math.abs(pose.getZ()) > maxZError) return "Robot height";
    if (Math.abs(pose.getRotation().getX()) > maxTiltRadians
        || Math.abs(pose.getRotation().getY()) > maxTiltRadians) return "Robot tilt";
    if (pose.getX() < 0
        || pose.getX() > aprilTagLayout.getFieldLength()
        || pose.getY() < 0
        || pose.getY() > aprilTagLayout.getFieldWidth()) return "Outside field";
    return "";
  }

  static Matrix<N3, N1> standardDeviations(PoseObservation observation, double cameraFactor) {
    double scale = Math.pow(observation.averageTagDistance(), 2) / observation.tagCount();
    double xy = Math.max(minLinearStdDev, linearStdDevBaseline * scale) * cameraFactor;
    double theta = Math.max(minAngularStdDev, angularStdDevBaseline * scale) * cameraFactor;
    if (!useVisionRotation || (observation.tagCount() == 1 && !useVisionRotationSingleTag))
      theta = ignoredAngularStdDev;
    return VecBuilder.fill(xy, xy, theta);
  }

  public boolean isConnected() {
    return cameras.length > 0 && Arrays.stream(cameras).allMatch(c -> c.inputs.connected);
  }

  /** A recent accepted pose, including legitimate poses at X = 0. */
  public Optional<Pose2d> getPose() {
    if (latestAccepted == null) return Optional.empty();
    double age = clock.getAsDouble() - latestAccepted.timestamp();
    return age >= -futureTimestampToleranceSeconds && age <= resetMaxAgeSeconds
        ? Optional.of(latestAccepted.pose().toPose2d())
        : Optional.empty();
  }

  /** Explicit operator reset; preserves gyro heading when the solve's heading is not trusted. */
  public void hardResetDrivePose() {
    var pose = getPose();
    Logger.recordOutput("Vision/ResetApplied", pose.isPresent());
    if (pose.isEmpty()) return;
    var reset = pose.get();
    if (!useVisionRotation || (latestAccepted.tagCount() == 1 && !useVisionRotationSingleTag)) {
      reset = new Pose2d(reset.getTranslation(), robotPose.get().getRotation());
    }
    resetPose.accept(reset);
    lastAppliedTimestamp = clock.getAsDouble();
    latestAccepted = null;
  }

  @Override
  public void close() {
    for (var camera : cameras) {
      camera.io.close();
      camera.disconnected.set(false);
    }
    rewind.close();
    CommandScheduler.getInstance().unregisterSubsystem(this);
  }

  @FunctionalInterface
  public interface VisionConsumer {
    void accept(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs);
  }

  private record Candidate(CameraState camera, PoseObservation observation) {}

  private static class CameraState {
    final CameraConfig config;
    final VisionIO io;
    final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    final String path;
    final Alert disconnected;
    double lastTimestamp = Double.NEGATIVE_INFINITY;
    final List<Pose3d> tags = new ArrayList<>(),
        poses = new ArrayList<>(),
        accepted = new ArrayList<>(),
        rejected = new ArrayList<>(),
        applied = new ArrayList<>();
    final List<String> reasons = new ArrayList<>();
    final List<Double> ages = new ArrayList<>(), appliedStdDevs = new ArrayList<>();

    CameraState(CameraConfig config, VisionIO io) {
      this.config = config;
      this.io = io;
      path = "Vision/" + config.name();
      disconnected =
          new Alert("Vision camera " + config.name() + " is disconnected.", AlertType.kWarning);
    }

    void clear() {
      tags.clear();
      poses.clear();
      accepted.clear();
      rejected.clear();
      applied.clear();
      reasons.clear();
      ages.clear();
      appliedStdDevs.clear();
    }

    void log(Pose3d robotPose) {
      Logger.recordOutput(path + "/TagPoses", tags.toArray(Pose3d[]::new));
      Logger.recordOutput(path + "/RobotPoses", poses.toArray(Pose3d[]::new));
      Logger.recordOutput(path + "/RobotPosesAccepted", accepted.toArray(Pose3d[]::new));
      Logger.recordOutput(path + "/RobotPosesRejected", rejected.toArray(Pose3d[]::new));
      Logger.recordOutput(path + "/RobotPosesApplied", applied.toArray(Pose3d[]::new));
      // Reasons and ages align with RobotPoses; std deviations are x/y/theta per applied pose.
      Logger.recordOutput(path + "/Decisions", reasons.toArray(String[]::new));
      Logger.recordOutput(
          path + "/MeasurementAgeSeconds",
          ages.stream().mapToDouble(Double::doubleValue).toArray());
      Logger.recordOutput(
          path + "/AppliedStdDevs",
          appliedStdDevs.stream().mapToDouble(Double::doubleValue).toArray());
      Logger.recordOutput(
          path + "/RobotToCameraPose", new Pose3d().transformBy(config.robotToCamera()));
      Logger.recordOutput(path + "/CameraPose", robotPose.transformBy(config.robotToCamera()));
    }
  }
}
