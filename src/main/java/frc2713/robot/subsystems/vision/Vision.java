package frc2713.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radian;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2713.robot.RobotContainer;
import frc2713.robot.subsystems.vision.VisionConstants.PoseEstimatorErrorStDevs;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private static final double MICROS_TO_SECONDS = 1.0e-6;
  private static final String PERIODIC_TIMING_KEY = "LoopTiming/Subsystems/Vision/PeriodicSec";

  private final VisionIO io;
  private final VisionInputsAutoLogged inputs;
  private Matrix<N3, N1> cachedVisionStdDevMatrix = null;
  private double lastTranslationStdDevMeters = Double.NaN;
  private double lastRotationStdDevRadians = Double.NaN;

  public Vision(VisionIO io) {
    this.io = io;
    inputs = new VisionInputsAutoLogged();
  }

  @Override
  public void periodic() {
    long periodicStartMicros = RobotController.getFPGATime();
    try {
      io.updateInputs(inputs);
      Logger.processInputs("Vision", inputs);

      if (inputs.applying && DriverStation.isEnabled()) {
        double translationStdDevMeters = inputs.translationStdDev.in(Meters);
        double rotationStdDevRadians = inputs.rotationStdDev.in(Radian);
        if (cachedVisionStdDevMatrix == null
            || translationStdDevMeters != lastTranslationStdDevMeters
            || rotationStdDevRadians != lastRotationStdDevRadians) {
          cachedVisionStdDevMatrix =
              new PoseEstimatorErrorStDevs(inputs.translationStdDev, inputs.rotationStdDev)
                  .toMatrix();
          lastTranslationStdDevMeters = translationStdDevMeters;
          lastRotationStdDevRadians = rotationStdDevRadians;
        }

        RobotContainer.drive.addVisionMeasurement(
            inputs.pose, inputs.timestamp, cachedVisionStdDevMatrix);
      }
    } finally {
      Logger.recordOutput(
          PERIODIC_TIMING_KEY, (RobotController.getFPGATime() - periodicStartMicros) * MICROS_TO_SECONDS);
    }
  }

  public Optional<Pose2d> getPose() {
    // a hack to not get the pose at init or sim
    if (inputs.pose.getTranslation().getX() != 0) {
      return Optional.of(inputs.pose);
    } else {
      return Optional.empty();
    }
  }
}
