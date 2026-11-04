package frc2713.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc2713.lib.util.LoggedTunableMeasure;
import java.util.Map;

public class VisionConstants {
  public static Distance MAX_POSE_JUMP = Meters.of(1);
  public static LinearVelocity MAX_LINEAR_SPEED =
      MetersPerSecond.of(0.02); // TODO: reasoning based on linear speed
  public static AngularVelocity MAX_ANGULAR_SPEED =
      DegreesPerSecond.of(30); // TODO: reasoning based on angular speed

  public record PoseEstimatorErrorStDevs(Distance translationalStDev, Angle rotationalStDev) {
    public PoseEstimatorErrorStDevs multiplyByRange(double range) {
      return new PoseEstimatorErrorStDevs(this.translationalStDev.times(range), rotationalStDev);
    }

    public Matrix<N3, N1> toMatrix() {
      return VecBuilder.fill(
          this.translationalStDev.in(Meters),
          this.translationalStDev.in(Meters),
          this.rotationalStDev.in(Radian));
    }
  }

  // Use a very large rotational standard deviation so that the pose estimator
  // effectively ignores the rotation component of vision measurements and relies
  // on other sensors (e.g., gyro/odometry) for heading, since vision rotation
  // data is considered too unreliable/noisy for this robot.
  public static final PoseEstimatorErrorStDevs POSE_ESTIMATOR_STATE_STDEVS =
      new PoseEstimatorErrorStDevs(Meters.of(0.01), Degrees.of(5));

  public static PoseEstimatorErrorStDevs POSE_ESTIMATOR_STATE_LOW_TAGS_FAST_STDEVS =
      new PoseEstimatorErrorStDevs(Meters.of(0.8), Degrees.of(999));

  public static PoseEstimatorErrorStDevs POSE_ESTIMATOR_STATE_LOW_TAGS_SLOW_STDEVS =
      new PoseEstimatorErrorStDevs(Meters.of(0.3), Degrees.of(999));

  public static final LoggedTunableMeasure<Time> LATENCY_THRESHOLD =
      new LoggedTunableMeasure<Time>("Vision/latencyThreshold", Seconds.of(0.2));

  // Names must match PhotonVision exactly.
  // Fixed robot-to-camera mounts in WPILib coordinates: +X forward, +Y left, +Z up (meters).
  // Rotation3d takes roll, pitch, yaw; negative pitch aims up, positive yaw turns left.
  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html
  // The old SLAMDUNK robot frame had +Y forward and +X right: (x, y, z) -> (y, -x, z).
  // Rotations also account for the optical camera frame (+Z out, +X right, +Y down) becoming
  // PhotonVision's +X out, +Y left, +Z up. The right and back cameras retain their 180-degree roll.
  public static final Map<String, Transform3d> CAMERA_POSES =
      Map.of(
          "front_1",
          new Transform3d(
              0.302558,
              -0.063500,
              0.502178,
              new Rotation3d(Degrees.of(0), Degrees.of(-10), Degrees.of(0))),
          "right_2",
          new Transform3d(
              -0.149225,
              -0.362883,
              0.518053,
              new Rotation3d(Degrees.of(180), Degrees.of(-17.5), Degrees.of(-90))),
          "back_3",
          new Transform3d(
              -0.313595,
              0.0,
              0.516118,
              new Rotation3d(Degrees.of(180), Degrees.of(-17.5), Degrees.of(180))),
          "left_4",
          new Transform3d(
              -0.149225,
              0.362883,
              0.518053,
              new Rotation3d(Degrees.of(0), Degrees.of(-10), Degrees.of(90))));

  // Record every camera to the Jetson's SSD (Rewind) from auto start until teleop ends
  public static final boolean REWIND_ENABLED = true;

  public static final String SLAMDUNK_IP = "10.27.13.201";
  public static final String SUPERCAP_IPC_ADDRESS = "tcp://" + SLAMDUNK_IP + ":5809";
}
