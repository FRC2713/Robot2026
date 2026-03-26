package frc2713.robot.subsystems.vision;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  public void setGyroAngle(Angle angle) {
    io.setGyroAngle(angle);
  }

  public Command setGyroAngleCmd(Angle angle) {
    return Commands.runOnce(() -> setGyroAngle(angle));
  }

  // Overload to set gyro angle from trajectory's initial pose
  public Command setGyroAngleCmd(AutoTrajectory traj) {
    return Commands.runOnce(
        () -> setGyroAngle(traj.getInitialPose().get().getRotation().getMeasure()));
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
