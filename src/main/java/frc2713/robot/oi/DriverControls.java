package frc2713.robot.oi;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc2713.robot.GameCommandGroups;
import frc2713.robot.commands.DriveCommands;
import frc2713.robot.subsystems.drive.Drive;
import frc2713.robot.subsystems.intake.IntakeExtension;
import frc2713.robot.subsystems.intake.IntakeRoller;
import frc2713.robot.subsystems.launcher.Flywheels;
import frc2713.robot.subsystems.launcher.Hood;
import frc2713.robot.subsystems.launcher.LaunchingSolutionManager;
import frc2713.robot.subsystems.launcher.Turret;
import frc2713.robot.subsystems.serializer.DyeRotor;
import frc2713.robot.subsystems.serializer.Feeder;

public class DriverControls {
  private final CommandVader4Controller controller = new CommandVader4Controller(0);

  private final Drive drive;
  private final Flywheels flywheels;
  private final Turret turret;
  private final Hood hood;
  private final IntakeRoller intakeRoller;
  private final IntakeExtension intakeExtension;
  private final DyeRotor dyeRotor;
  private final Feeder feeder;

  public DriverControls(
      Drive drive,
      Flywheels flywheels,
      Turret turret,
      Hood hood,
      IntakeRoller intakeRollers,
      IntakeExtension intakeExtension,
      DyeRotor dyeRotor,
      Feeder feeder) {
    this.drive = drive;
    this.flywheels = flywheels;
    this.turret = turret;
    this.hood = hood;
    this.intakeRoller = intakeRollers;
    this.intakeExtension = intakeExtension;
    this.dyeRotor = dyeRotor;
    this.feeder = feeder;
  }

  public void configureButtonBindings() {

    // Reset gyro to 0 deg when start button is pressed
    controller
        .start()
        .onTrue(
            Commands.parallel(
                this.setToNormalDriveCmd(),
                Commands.runOnce(
                        () ->
                            drive.setPose(
                                new Pose2d(
                                    drive.getPose().getTranslation(), Rotation2d.fromDegrees(0))),
                        drive)
                    .ignoringDisable(true)));

    // // Reset gyro to 180 deg when start button is pressed
    controller
        .back()
        .onTrue(
            Commands.parallel(
                    this.setToNormalDriveCmd(),
                    Commands.runOnce(
                        () ->
                            drive.setPose(
                                new Pose2d(
                                    drive.getPose().getTranslation(), Rotation2d.fromDegrees(180))),
                        drive))
                .ignoringDisable(true));

    // POV Precision Driving
    controller
        .povLeft()
        .onTrue(
            DriveCommands.changeDefaultDriveCommand(
                drive, DriveCommands.inch(drive, DriveCommands.INCH_SPEED), "Inch Left"))
        .onFalse(this.setToNormalDriveCmd());
    controller
        .povRight()
        .onTrue(
            DriveCommands.changeDefaultDriveCommand(
                drive,
                DriveCommands.inch(drive, () -> -1 * DriveCommands.INCH_SPEED.getAsDouble()),
                "Inch Right"))
        .onFalse(this.setToNormalDriveCmd());

    // intake fuel
    controller
        .leftTrigger(0.98)
        .onTrue(
            Commands.parallel(
                    intakeExtension.extendCommand(),
                    Commands.parallel(intakeRoller.intake(), dyeRotor.stirFuel()))
                .withName("Intaking"))
        .onFalse(Commands.parallel(intakeRoller.stop(), dyeRotor.stop()).withName("Stop Intake"));

    controller
        .leftBumper()
        .onTrue(
            Commands.parallel(intakeExtension.retractCommand(), intakeRoller.intake())
                .withName("Retract Intake"))
        .onFalse(intakeRoller.stop().withName("Stop Intake"));
    // intake align
    controller
        .a()
        .whileTrue(
            DriveCommands.changeDefaultDriveCommand(
                drive,
                GameCommandGroups.intakeAlign(
                    drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()),
                "Drive with Intake Align"))
        .onFalse(setToNormalDriveCmd())
        .onTrue(
            Commands.runOnce(
                () -> LaunchingSolutionManager.ZoneSelectionHelpers.setIntakeRotation()));

    // shoot otf
    controller
        .rightBumper()
        .whileTrue(
            GameCommandGroups.Launching.otfShotHoodProtect(
                    drive, flywheels, hood, turret, feeder, dyeRotor, intakeExtension, intakeRoller)
                .withName("OTF Shooting"))
        .onFalse(
            Commands.parallel(
                    GameCommandGroups.Launching.stopShootingAndRetractHood(
                        drive, feeder, dyeRotor, hood, flywheels))
                .withName("Stop Shooting"));

    // controller
    //     .a()
    //     .whileFalse(DriveCommands.changeDefaultDriveCommand(drive,
    //             DriveCommands.joystickDriveAtAngle(drive,
    //              () -> driverControls.getLeftY() + devControls.getLeftY(),
    //              () -> driverControls.getLeftX() + devControls.getLeftX(),
    //              () -> ), null))

    // shoot when flywheels are ready
    controller
        .rightTrigger(.98)
        .whileTrue(
            GameCommandGroups.Launching.dumbShot(
                    drive, flywheels, hood, turret, feeder, dyeRotor, intakeExtension, intakeRoller)
                .withName("Dumb Shooting"))
        .onFalse(
            GameCommandGroups.Launching.stopShootingAndRetractHood(
                    drive, feeder, dyeRotor, hood, flywheels)
                .withName("Stop Shooting + Hood Retract"));

    // controller
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
    DriveCommands.setDefaultDriveCommand(drive, this.normalDriveCmd(), "Default Joystick Drive");
  }

  public Command setToNormalDriveCmd() {
    return DriveCommands.changeDefaultDriveCommand(
        drive, this.normalDriveCmd(), "Default Joystick Drive");
  }

  public Command stopWithX() {
    return DriveCommands.changeDefaultDriveCommand(
        drive, new InstantCommand(() -> drive.stopWithX()), "Stop With X");
  }

  private Command normalDriveCmd() {
    return DriveCommands.joystickDrive(
        drive,
        () -> -controller.getLeftY(),
        () -> -controller.getLeftX(),
        () -> -controller.getRightX());
  }
}
