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
import edu.wpi.first.math.geometry.Translation3d;
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

  /**
   * Convert SLAMDUNK axis-angle rotation vector to WPILib Rotation3d. The rotation vector's
   * direction is the axis and its magnitude is the angle in radians.
   */
  private static Rotation3d fromAxisAngle(double rx, double ry, double rz) {
    double angle = Math.sqrt(rx * rx + ry * ry + rz * rz);
    if (angle < 1e-9) return new Rotation3d();
    // Rotation3d(axis, angle) - axis is normalized internally
    return new Rotation3d(VecBuilder.fill(rx / angle, ry / angle, rz / angle), angle);
  }

  // Names must match PhotonVision exactly.
  // Poses from SLAMDUNK - rotation vectors are axis-angle
  public static final Map<String, Transform3d> CAMERA_POSES =
      Map.of(
          //   "front_1",
          //   new Transform3d(
          //       new Translation3d(0.063500, 0.302558, 0.502178),
          //       fromAxisAngle(-1.3962634015954634, 0, 0)),
          //   "right_2",
          //   new Transform3d(
          //       new Translation3d(0.362883, -0.149225, 0.518053),
          //       fromAxisAngle(0.9813180646681502, 0.9813180646681504, 1.338348969126305)),
          //   "back_3",
          //   new Transform3d(
          //       new Translation3d(0, -0.313595, 0.516118),
          //       fromAxisAngle(1.2653637076958888, 0, 0)),
          //   "left_4",
          //   new Transform3d(
          //       new Translation3d(-0.362883, -0.149225, 0.518053),
          //       fromAxisAngle(-1.0796670529547816, -1.0796670529547816, 1.2866970891644645)),
          "spare_thrifty",
          new Transform3d(
              new Translation3d(0.063500, 0.302558, 0.502178),
              fromAxisAngle(-1.3962634015954634, 0, 0)));

  // Record every camera to the Jetson's SSD (Rewind) from auto start until teleop ends
  public static final boolean REWIND_ENABLED = true;

  public static final String SLAMDUNK_IP = "10.27.13.201";
  public static final String SUPERCAP_IPC_ADDRESS = "tcp://" + SLAMDUNK_IP + ":5809";
}
