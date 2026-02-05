package frc2713.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radian;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class VisionConstants {
  public static double MAX_POSE_JUMP_METERS = 2;
  public static double MAX_SPEED = 0.02;

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

  public static PoseEstimatorErrorStDevs POSE_ESTIMATOR_STATE_STDEVS =
      new PoseEstimatorErrorStDevs(Meters.of(0.01), Degrees.of(999));
}
