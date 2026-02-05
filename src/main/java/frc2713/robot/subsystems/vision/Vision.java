package frc2713.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2713.robot.RobotContainer;
import frc2713.robot.subsystems.vision.VisionConstants.PoseEstimatorErrorStDevs;
import frc2713.robot.subsystems.vision.VisionIO.VisionInputs;
import java.util.Optional;

public class Vision extends SubsystemBase {

  private final VisionIO io;
  private final VisionInputs inputs;

  public Vision(VisionIO io) {
    this.io = io;
    inputs = new VisionInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.update(inputs);
    if (inputs.applying && DriverStation.isEnabled()) {
      RobotContainer.drive.addVisionMeasurement(
          inputs.pose,
          inputs.timestamp,
          new PoseEstimatorErrorStDevs(inputs.translationStdDev, inputs.rotationStdDev).toMatrix());
    }
  }

  public Optional<Pose2d> getPose() {
    if (inputs.pose.getTranslation().getX() != 0) {
      return Optional.of(inputs.pose);
    } else {
      return Optional.empty();
    }
  }
}
