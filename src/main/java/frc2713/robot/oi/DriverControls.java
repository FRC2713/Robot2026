package frc2713.robot.oi;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc2713.robot.RobotContainer;
import frc2713.robot.commands.DriveCommands;

public class DriverControls {
  private final CommandXboxController controller = new CommandXboxController(0);


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

  
    // controller
    //     .rightBumper()
    //     .onTrue(flywheels.velocitySetpointCommand(LauncherConstants.Flywheels.PIDTest))
    //     .onFalse(flywheels.velocitySetpointCommand(() -> RPM.of(0)));

    controller
        .rightBumper()
        .whileTrue(
            Commands.parallel(intakeRoller.intake(), intakeExtension.extendCommand())
                .withName("Intaking"))
        .onFalse(
            Commands.parallel(intakeRoller.stop(), intakeExtension.retractCommand())
                .withName("Intake Idle"));

    controller
        .leftTrigger(0.25)
        .whileTrue(
            Commands.parallel(
                    flywheels.otfCommand(),
                    hood.otfCommand(),
                    turret.oftCommand(),
                    flywheels.simulateLaunchedFuel(
                        () -> {
                          return flywheels.atTarget() && hood.atTarget() && turret.atTarget();
                        }),
                    feeder.feedWhenReady(
                        () -> {
                          return flywheels.atTarget() && hood.atTarget() && turret.atTarget();
                        }),
                    dyeRotor.feedWhenReady(
                        () -> {
                          return flywheels.atTarget() && hood.atTarget() && turret.atTarget();
                        }))
                .withName("OTF Shooting"));

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
