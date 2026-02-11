package frc2713.robot.oi;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc2713.robot.commands.DriveCommands;
import frc2713.robot.subsystems.drive.Drive;
import frc2713.robot.subsystems.intake.IntakeExtension;
import frc2713.robot.subsystems.intake.IntakeRoller;
import frc2713.robot.subsystems.launcher.Flywheels;
import frc2713.robot.subsystems.launcher.Hood;
import frc2713.robot.subsystems.launcher.Turret;
import frc2713.robot.subsystems.serializer.DyeRotor;
import frc2713.robot.subsystems.serializer.Feeder;

@SuppressWarnings("unused")
public class DevControls {
  private final CommandXboxController controller = new CommandXboxController(1);

  private final Drive drive;
  private final Flywheels flywheels;
  private final Turret turret;
  private final Hood hood;
  private final IntakeRoller intakeRoller;
  private final IntakeExtension intakeExtension;
  private final DyeRotor dyeRotor;
  private final Feeder feeder;

  public DevControls(
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

    // Messes with dualsense controls
    // Reset gyro to 0 deg when start button is pressed
    // controller
    //     .start()
    //     .onTrue(
    //         Commands.parallel(
    //             this.setToNormalDriveCmd(),
    //             Commands.runOnce(
    //                     () ->
    //                         drive.setPose(
    //                             new Pose2d(
    //                                 drive.getPose().getTranslation(),
    // Rotation2d.fromDegrees(0))),
    //                     drive)
    //                 .ignoringDisable(true)));

    // // Reset gyro to 180 deg when start button is pressed
    // controller
    //     .back()
    //     .onTrue(
    //         Commands.parallel(
    //                 this.setToNormalDriveCmd(),
    //                 Commands.runOnce(
    //                     () ->
    //                         drive.setPose(
    //                             new Pose2d(
    //                                 drive.getPose().getTranslation(),
    // Rotation2d.fromDegrees(180))),
    //                     drive))
    //             .ignoringDisable(true));

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

    // Turret angle controls
    // Manual turret rotation with triggers - continuously target far in the desired direction
    // Velocity and acceleration scale with how hard the trigger is pressed
    controller
        .leftTrigger(0.01)
        .whileTrue(
            turret.setAngleStopAtBounds(
                () -> Degrees.of(turret.getComputedTurretPosition().in(Degrees) + 180),
                controller::getLeftTriggerAxis));

    controller
        .rightTrigger(0.01)
        .whileTrue(
            turret.setAngleStopAtBounds(
                () -> Degrees.of(turret.getComputedTurretPosition().in(Degrees) - 180),
                controller::getRightTriggerAxis));

    // Hood manual controls - bumpers bring hood up/down continuously until bounds hit
    controller
        .leftBumper()
        .whileTrue(
            hood.setAngleStopAtBounds(
                () -> Degrees.of(hood.getCurrentPosition().in(Degrees) - 5))); // Bring hood down

    controller
        .rightBumper()
        .whileTrue(
            hood.setAngleStopAtBounds(
                () -> Degrees.of(hood.getCurrentPosition().in(Degrees) + 5))); // Bring hood up
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
