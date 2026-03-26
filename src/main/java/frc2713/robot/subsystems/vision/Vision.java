package frc2713.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2713.lib.logging.PeriodicTimingLogger;
import frc2713.lib.logging.TimeLogged;
import frc2713.robot.RobotContainer;
import frc2713.robot.subsystems.vision.VisionConstants.PoseEstimatorErrorStDevs;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

  private final VisionIO io;
  private final VisionInputsAutoLogged inputs;

  private final Field2d loggedPoseOnField = new Field2d(); // for elastic layout

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
            new PoseEstimatorErrorStDevs(inputs.translationStdDev, inputs.rotationStdDev)
                .toMatrix());

        loggedPoseOnField.setRobotPose(inputs.pose);
      }

      Logger.processInputs("Vision", inputs);
      SmartDashboard.putData("Vision/robot_on_field", loggedPoseOnField);
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
