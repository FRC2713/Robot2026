package frc2713.robot.commands.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc2713.robot.GameCommandGroups;
import frc2713.robot.RobotContainer;
import frc2713.robot.subsystems.drive.Drive;
import frc2713.robot.subsystems.intake.IntakeExtension;
import frc2713.robot.subsystems.intake.IntakeRoller;
import frc2713.robot.subsystems.launcher.Flywheels;
import frc2713.robot.subsystems.launcher.Hood;
import frc2713.robot.subsystems.launcher.Turret;
import frc2713.robot.subsystems.serializer.DyeRotor;
import frc2713.robot.subsystems.serializer.Feeder;
import frc2713.robot.util.AutoUtil;

/**
 * Starts at right trench. collects from Neutral Zone once. Goes back to right trench. Shots while
 * stationary. Goes to Neutral Zone and waits for teleop.
 */
public class Midwars {

  public static Command getRoutine(
      AutoFactory factory,
      boolean doFlip,
      Drive driveSubsystem,
      IntakeExtension intakeExtension,
      IntakeRoller intakeRoller,
      Flywheels flywheels,
      Hood hood,
      Turret turret,
      DyeRotor dyeRotor,
      Feeder feeder) {
    AutoRoutine routine = factory.newRoutine("Midwars");

    AutoTrajectory intakeFuelRight =
        AutoUtil.flipHorizontalIf(doFlip, routine.trajectory("IntakeFuelRight"), routine);
    AutoTrajectory moveToLaunchRight =
        AutoUtil.flipHorizontalIf(doFlip, routine.trajectory("MoveToLaunchRight"), routine);
    AutoTrajectory intakeFuelRight2 =
        AutoUtil.flipHorizontalIf(doFlip, routine.trajectory("IntakeFuelRight2"), routine);
    AutoTrajectory moveToLaunchRight2 =
        AutoUtil.flipHorizontalIf(doFlip, routine.trajectory("BumpToLaunchRight"), routine);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                Commands.print("[AUTO] Going to fuel"),
                intakeFuelRight.resetOdometry(),
                RobotContainer.vision.setGyroAngleCmd(intakeFuelRight),
                intakeFuelRight.cmd()));

    intakeFuelRight
        .atTime("BeginIntaking")
        .onTrue(
            Commands.parallel(
                Commands.print("[AUTO] Marker-based intake start"),
                intakeExtension.extendCommand(),
                intakeRoller.intake()));

    intakeFuelRight
        .done()
        .onTrue(
            Commands.parallel(
                Commands.print("[AUTO] Moving to launch position"),
                intakeRoller.intake(),
                flywheels.otfCommand(),
                moveToLaunchRight.cmd()));

    moveToLaunchRight
        .done()
        .onTrue(
            // SequentialCommandGroups inherit the requirements of their elements. So this whole
            // sequence has the requirements that otfShotSupplier.get(), which is everything.
            // This means that if one of our subsystems tries to run a command while the
            // intakeFuelRight2 path is running, the intakeFuelRight2 path will get interuppted,
            // along with the entire sequence.
            // That's why we can't use duck during autos or use the event marker to bring out the
            // intake after otfSupplier has been used.
            Commands.sequence(
                    Commands.print("[AUTO] Starting launch sequence"),
                    Commands.runOnce(driveSubsystem::stop),
                    GameCommandGroups.Launching.autoOtfShot(
                            driveSubsystem,
                            flywheels,
                            hood,
                            turret,
                            feeder,
                            dyeRotor,
                            intakeExtension,
                            intakeRoller)
                        .withTimeout(5),
                    GameCommandGroups.Launching.stopShootingAndRetractHood(
                            driveSubsystem, feeder, dyeRotor, hood, flywheels)
                        .withTimeout(0.25),
                    Commands.print("[AUTO] Going to fuel again"),
                    Commands.parallel(
                        // Wait to extend as moving to intake position
                        Commands.parallel(intakeExtension.extendCommand(), intakeRoller.intake()),
                        intakeFuelRight2.cmd()))
                .handleInterrupt(
                    () -> System.out.println("[AUTO] IntakeFuelRight2 likely got interuppted")));

    intakeFuelRight2
        .done()
        .onTrue(
            Commands.sequence(
                Commands.print("[AUTO] Going to launch again"),
                Commands.parallel(
                    flywheels.otfCommand(),
                    intakeExtension.extendCommand(),
                    intakeRoller.intake(),
                    moveToLaunchRight2.cmd())));

    // intakeFuelRight2
    //     .atTranslation("BeginIntaking", 0.2)
    //     .onTrue(
    //         Commands.parallel(
    //             Commands.print("[AUTO] Marker-based intake start2"),
    //             intakeExtension.extendCommand(),
    //             intakeRoller.intake()));

    moveToLaunchRight2
        .done()
        .onTrue(
            Commands.sequence(
                Commands.print("[AUTO] Starting launch sequence"),
                Commands.sequence(
                        Commands.parallel(
                            Commands.run(() -> driveSubsystem.stop())
                                .withDeadline(new WaitCommand(9)),
                            Commands.race(
                                GameCommandGroups.Launching.autoOtfShot(
                                    driveSubsystem,
                                    flywheels,
                                    hood,
                                    turret,
                                    feeder,
                                    dyeRotor,
                                    intakeExtension,
                                    intakeRoller),
                                new WaitCommand(9))),
                        Commands.sequence(
                            hood.retract().withTimeout(0.1),
                            Commands.parallel(
                                GameCommandGroups.Launching.stopShooting(
                                    driveSubsystem, feeder, dyeRotor, flywheels),
                                Commands.run(() -> driveSubsystem.stop()))))
                    .withName("OTF Shooting")));
    return routine.cmd();
  }
}
