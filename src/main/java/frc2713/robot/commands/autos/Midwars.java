package frc2713.robot.commands.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc2713.robot.GameCommandGroups;
import frc2713.robot.subsystems.drive.Drive;
import frc2713.robot.subsystems.intake.IntakeExtension;
import frc2713.robot.subsystems.intake.IntakeRoller;
import frc2713.robot.subsystems.launcher.Flywheels;
import frc2713.robot.subsystems.launcher.Hood;
import frc2713.robot.subsystems.launcher.Turret;
import frc2713.robot.subsystems.serializer.DyeRotor;
import frc2713.robot.subsystems.serializer.Feeder;
import java.util.function.Supplier;

/**
 * Starts at right trench. collects from Neutral Zone once. Goes back to right trench. Shots while
 * stationary. Goes to Neutral Zone and waits for teleop.
 */
public class Midwars {
  public static AutoRoutine getRoutine(
      AutoFactory factory,
      Drive driveSubsystem,
      IntakeExtension intakeExtension,
      IntakeRoller intakeRoller,
      Flywheels flywheels,
      Hood hood,
      Turret turret,
      DyeRotor dyeRotor,
      //   Launcher intakeAndShooter,
      Feeder feeder,
      Supplier<Command> otfShotSupplier) {
    AutoRoutine routine = factory.newRoutine("Midwars");

    AutoTrajectory intakeFuelRight = routine.trajectory("IntakeFuelRight");

    AutoTrajectory moveToLaunchRight = routine.trajectory("MoveToLaunchRight");
    AutoTrajectory intakeFuelRight2 = routine.trajectory("IntakeFuelRight2");
    AutoTrajectory moveToLaunchRight2 = routine.trajectory("MoveToLaunchRight");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                Commands.print("[AUTO] Going to fuel"),
                intakeFuelRight.resetOdometry(),
                intakeFuelRight.cmd()));

    intakeFuelRight
        .atTranslation("BeginIntaking", 0.2)
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
                moveToLaunchRight.cmd()));

    moveToLaunchRight
        .done()
        .onTrue(
            Commands.sequence(
                Commands.print("[AUTO] Starting launch sequence"),
                Commands.sequence(
                        Commands.runOnce(driveSubsystem::stop),
                        otfShotSupplier.get().withTimeout(7),
                        Commands.parallel(
                            GameCommandGroups.Launching.stopShooting(
                                driveSubsystem, feeder, dyeRotor, flywheels),
                            // Commands.parallel(
                            //     Commands.run(() -> driveSubsystem.stop()).withTimeout(6),
                            //     otfShotSupplier.get().withTimeout(6)),
                            // Commands.parallel(
                            Commands.print("[AUTO] Stopping launch sequence"),
                            hood.retract(),
                            intakeFuelRight2
                                .cmd()
                                .handleInterrupt(
                                    () ->
                                        System.out.println("[AUTO] debug, path got interuppted"))))
                    .withName("OTF Shooting")));

    intakeFuelRight2
        .done()
        .onTrue(
            Commands.sequence(
                Commands.print("[AUTO] Going to fuel again"),
                Commands.parallel(
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
                                .withDeadline(new WaitCommand(6)),
                            Commands.race(otfShotSupplier.get(), new WaitCommand(6))),
                        Commands.sequence(
                            hood.retract().withTimeout(0.1),
                            Commands.parallel(
                                GameCommandGroups.Launching.stopShooting(
                                    driveSubsystem, feeder, dyeRotor, flywheels),
                                Commands.run(() -> driveSubsystem.stop()))))
                    .withName("OTF Shooting")));
    return routine;
  }

  public static Command routine(
      AutoFactory factory,
      Drive driveSubsystem,
      IntakeExtension intakeExtension,
      IntakeRoller intakeRoller,
      Flywheels flywheels,
      Hood hood,
      Turret turret,
      DyeRotor dyeRotor,
      Feeder feeder,
      Supplier<Command> otfShotSupplier) {
    return Midwars.getRoutine(
            factory,
            driveSubsystem,
            intakeExtension,
            intakeRoller,
            flywheels,
            hood,
            turret,
            dyeRotor,
            feeder,
            otfShotSupplier)
        .cmd();
  }
}
