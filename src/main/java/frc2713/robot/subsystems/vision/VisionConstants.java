package frc2713.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class VisionConstants {
  public static Distance MAX_POSE_JUMP = Meters.of(2);
  public static LinearVelocity MAX_SPEED = MetersPerSecond.of(0.02);

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
  public static PoseEstimatorErrorStDevs POSE_ESTIMATOR_STATE_STDEVS =
      new PoseEstimatorErrorStDevs(Meters.of(0.01), Degrees.of(999));
}
