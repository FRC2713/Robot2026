package frc2713.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2713.robot.FieldConstants;
import frc2713.lib.logging.PeriodicTimingLogger;
import frc2713.lib.logging.TimeLogged;
import frc2713.robot.RobotContainer;
import frc2713.robot.subsystems.vision.VisionConstants.PoseEstimatorErrorStDevs;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

  private final VisionIO io;
  private final VisionInputsAutoLogged inputs;

  public Vision(VisionIO io) {
    this.io = io;
    inputs = new VisionInputsAutoLogged();
  }

  @Override
  @TimeLogged("Performance/SubsystemPeriodic/Vision")
  public void periodic() {
        try (var ignored = PeriodicTimingLogger.time(this)) {

    io.updateInputs(inputs);

    if (inputs.applying) {
      RobotContainer.drive.addVisionMeasurement(
          inputs.pose,
          inputs.timestamp,
          new PoseEstimatorErrorStDevs(inputs.translationStdDev, inputs.rotationStdDev).toMatrix());
    }
    Logger.processInputs("Vision", inputs);

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
