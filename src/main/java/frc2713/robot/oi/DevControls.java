package frc2713.robot.oi;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc2713.robot.RobotContainer;
import frc2713.robot.commands.DriveCommands;

public class DevControls {
  private final CommandXboxController controller = new CommandXboxController(1);

  public void configureButtonBindings() {

    // Reset gyro to 0 deg when start button is pressed
    controller
        .start()
        .onTrue(
            Commands.parallel(
                this.setToNormalDriveCmd(),
                Commands.runOnce(
                        () ->
                            RobotContainer.drive.setPose(
                                new Pose2d(
                                    RobotContainer.drive.getPose().getTranslation(),
                                    Rotation2d.fromDegrees(0))),
                        RobotContainer.drive)
                    .ignoringDisable(true)));

    // Reset gyro to 180 deg when start button is pressed
    controller
        .back()
        .onTrue(
            Commands.parallel(
                    this.setToNormalDriveCmd(),
                    Commands.runOnce(
                        () ->
                            RobotContainer.drive.setPose(
                                new Pose2d(
                                    RobotContainer.drive.getPose().getTranslation(),
                                    Rotation2d.fromDegrees(180))),
                        RobotContainer.drive))
                .ignoringDisable(true));

    // POV Precision Driving
    controller
        .povLeft()
        .onTrue(
            DriveCommands.changeDefaultDriveCommand(
                RobotContainer.drive,
                DriveCommands.inch(RobotContainer.drive, DriveCommands.INCH_SPEED),
                "Inch Left"))
        .onFalse(this.setToNormalDriveCmd());
    controller
        .povRight()
        .onTrue(
            DriveCommands.changeDefaultDriveCommand(
                RobotContainer.drive,
                DriveCommands.inch(
                    RobotContainer.drive, () -> -1 * DriveCommands.INCH_SPEED.getAsDouble()),
                "Inch Right"))
        .onFalse(this.setToNormalDriveCmd());

    // Turret angle controls
    controller.leftBumper().onTrue(RobotContainer.turret.setAngle(() -> Degrees.of(90)));
    controller.rightBumper().onTrue(RobotContainer.turret.setAngle(() -> Degrees.of(-270)));
    controller.leftTrigger().whileTrue(RobotContainer.turret.oftCommand());
  }

  public double getLeftY() {
    return controller.getLeftY();
  }

  public double getLeftX() {
    return controller.getLeftX();
  }

  public double getRightX() {
    return controller.getRightX();
  }

  public double getRightY() {
    return controller.getRightY();
  }

  public void setToNormalDrive() {
    DriveCommands.setDefaultDriveCommand(
        RobotContainer.drive, this.normalDriveCmd(), "Default Joystick Drive");
  }

  public Command setToNormalDriveCmd() {
    return DriveCommands.changeDefaultDriveCommand(
        RobotContainer.drive, this.normalDriveCmd(), "Default Joystick Drive");
  }

  public Command stopWithX() {
    return DriveCommands.changeDefaultDriveCommand(
        RobotContainer.drive,
        new InstantCommand(() -> RobotContainer.drive.stopWithX()),
        "Stop With X");
  }

  private Command normalDriveCmd() {
    return DriveCommands.joystickDrive(
        RobotContainer.drive,
        () -> -controller.getLeftY(),
        () -> -controller.getLeftX(),
        () -> -controller.getRightX());
  }
}
