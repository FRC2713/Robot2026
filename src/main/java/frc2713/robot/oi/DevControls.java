package frc2713.robot.oi;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.controls.RgbFadeAnimation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc2713.robot.RobotContainer;
import frc2713.robot.commands.DriveCommands;
import frc2713.robot.subsystems.drive.Drive;
import frc2713.robot.subsystems.intake.IntakeExtension;
import frc2713.robot.subsystems.intake.IntakeRoller;
import frc2713.robot.subsystems.launcher.Flywheels;
import frc2713.robot.subsystems.launcher.Hood;
import frc2713.robot.subsystems.launcher.LauncherConstants;
import frc2713.robot.subsystems.launcher.Turret;
import frc2713.robot.subsystems.led.LEDConstants;
import frc2713.robot.subsystems.serializer.DyeRotor;
import frc2713.robot.subsystems.serializer.Feeder;

@SuppressWarnings("unused")
public class DevControls {
  private final CommandVader4Controller controller = new CommandVader4Controller(2);

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
    controller.povLeft().onTrue(flywheels.stop());

    // Test setting drive limits
    // controller
    //     .a()
    //     .onTrue(
    //         DriveCommands.setDriveLimits(
    //             drive,
    //             Optional.of(FeetPerSecond.of(1.0)),
    //             Optional.of(FeetPerSecondPerSecond.of(12.0)),
    //             Optional.of(DegreesPerSecond.of(90.0)),
    //             Optional.of(DegreesPerSecondPerSecond.of(360.0))));
    controller.b().onTrue(DriveCommands.clearDriveLimits(drive));

    controller
        .povLeft()
        .onTrue(
            DriveCommands.joystickDriveAtAngle(
                    drive, () -> 0, () -> 0, () -> Rotation2d.fromDegrees(180))
                .withName("Drive Intake Align"));

    controller
        .povRight()
        .onTrue(
            DriveCommands.joystickDriveAtAngle(
                    drive, () -> 0, () -> 0, () -> Rotation2d.fromDegrees(0))
                .withName("Drive Intake Align"));

    controller.povUp().onTrue(DriveCommands.driveOneMeter(drive, 1).withName("Drive Intake Align"));

    controller
        .povDown()
        .onTrue(DriveCommands.driveOneMeter(drive, -1).withName("Drive Intake Align"));

    // Intake Controls

    controller
        .leftTrigger(0.25)
        .onTrue(
            Commands.parallel(
                    intakeExtension.extendCommand(),
                    Commands.sequence(
                        Commands.waitSeconds(0.5),
                        Commands.parallel(intakeRoller.intake(), dyeRotor.stirFuel())))
                .withName("Intaking"))
        .onFalse(Commands.parallel(intakeRoller.stop().withName("Stop Intake"), dyeRotor.stop()));

    controller
        .leftBumper()
        .onTrue(
            Commands.parallel(intakeExtension.retractCommand(), intakeRoller.intake())
                .withName("Retract Intake"))
        .onFalse(intakeRoller.stop().withName("Stop Intake"));

    // Hood Controls
    // controller.povUp().onTrue(hood.setAngleCommand(() -> Degrees.of(25)));

    // controller.povDown().onTrue(hood.setAngleCommand(() -> Degrees.of(0.5)));

    // controller
    //     .leftBumper()
    //     .whileTrue(hood.setAngleCommand(() -> hood.getCurrentPosition().minus(Degrees.of(2))));
    // controller
    //     .rightBumper()
    //     .whileTrue(hood.setAngleCommand(() -> hood.getCurrentPosition().plus(Degrees.of(2))));

    // Turret Controls

    // controller.a().whileTrue(turret.setAngleStopAtBounds(LauncherConstants.Turret.PIDTestAngleOne));

    // controller.b().whileTrue(turret.setAngleStopAtBounds(LauncherConstants.Turret.PIDTestAngleTwo));

    controller
        .a()
        .onTrue(
            RobotContainer.leds
                .setAnimation(
                    new RgbFadeAnimation(0, Math.max(0, LEDConstants.ledCount - 1)).withSlot(0))
                .ignoringDisable(true)
                .withName("Dev LED RGB Fade"));
    controller
        .rightBumper()
        .onTrue(flywheels.setVelocity(LauncherConstants.Flywheels.PIDTest))
        .onFalse(
            flywheels.setVelocity(
                () -> LauncherConstants.Flywheels.PIDTest.get().minus(RPM.of(1000))));
    // controller
    //     .b()
    //     .whileTrue(
    //         GameCommandGroups.Launching.otfShotHoodProtect(
    //             drive, flywheels, hood, turret, feeder, dyeRotor, intakeExtension, intakeRoller))
    //     .whileFalse(GameCommandGroups.Launching.stopShooting(drive, feeder, dyeRotor,
    // flywheels));

    // controller
    //     .x()
    //     .whileTrue(
    //         DriveCommands.changeDefaultDriveCommand(
    //             drive,
    //             GameCommandGroups.intakeAlign(
    //                 drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()),
    //             "Intake Align"))
    //     .onFalse(setToNormalDriveCmd())
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> LaunchingSolutionManager.ZoneSelectionHelpers.setIntakeRotation()));
    // Serializer control

    // A button - index fuel
    // controller.a().whileTrue(dyeRotor.indexFuel()).onFalse(dyeRotor.stopCommand());

    // B button - index fuel
    // controller.b().whileTrue(feeder.feedShooter()).onFalse(feeder.stop());

    // controller
    //     .x()
    //     .onTrue(flywheels.velocitySetpointCommand(LauncherConstants.Flywheels.PIDTest))
    //     .onFalse(flywheels.velocitySetpointCommand(() -> RPM.of(0)));

    // controller
    //     .x()
    //     .onTrue(Commands.runOnce(() ->
    // RobotContainer.drive.changeDriveCurrentLimits(Amps.of(70))));

    // controller
    //     .y()
    //     .onTrue(Commands.runOnce(() ->
    // RobotContainer.drive.changeDriveCurrentLimits(Amps.of(80))));

    // controller
    //     .x()
    //     .onTrue(flywheels.dutyCycleCommand(() -> 1.0))
    //     .onFalse(flywheels.dutyCycleCommand(() -> 0.0));

    // controller.povUp().onTrue(hood.setAngleCommand(() -> Degrees.of(25)));
    // controller.povDown().onTrue(hood.setAngleCommand(() -> Degrees.of(0)));

    // Y button - index fuel in parallel (same effect since it's the same command)
    // controller
    //     .y()
    //     .whileTrue(
    //         Commands.sequence(
    //             flywheels.setVelocityUntilTarget(
    //                 LauncherConstants.Flywheels
    //                     .PIDTest), // Spin up flywheels to a test launch velocity
    //             Commands.parallel(dyeRotor.indexFuel(), feeder.feedShooter())))
    //     .onFalse(Commands.parallel(dyeRotor.stop(), feeder.stop(), flywheels.stop()));

    // Shoot when flywheels are ready
  }

  public Command startRumble() {
    return Commands.run(
        () -> {
          controller.setRumble(GenericHID.RumbleType.kLeftRumble, 1.0);
          controller.setRumble(GenericHID.RumbleType.kRightRumble, 1.0);
        });
  }

  public Command stopRumble() {
    return Commands.run(
        () -> {
          controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
          controller.setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
        });
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

  public double getLeftTriggerAxis() {
    return controller.getLeftTriggerAxis();
  }

  public double getRightTriggerAxis() {
    return controller.getRightTriggerAxis();
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
