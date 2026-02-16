package frc2713.robot.commands.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc2713.robot.subsystems.drive.Drive;
import frc2713.robot.subsystems.intake.IntakeExtension;
import frc2713.robot.subsystems.intake.IntakeRoller;
import frc2713.robot.subsystems.launcher.Flywheels;
import frc2713.robot.subsystems.launcher.Hood;
import frc2713.robot.subsystems.launcher.Turret;
import frc2713.robot.subsystems.serializer.DyeRotor;
import frc2713.robot.subsystems.serializer.Feeder;
import java.util.function.Supplier;

public class NeutralScoreNeutral {
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
    AutoRoutine routine = factory.newRoutine("Start Collect Shoot");

    AutoTrajectory faceFuelTrench = routine.trajectory("FaceFuelTrench");
    AutoTrajectory intakeFuel = routine.trajectory("IntakeFuel");
    AutoTrajectory moveToLaunchTrench = routine.trajectory("MoveToLaunchTrench");
    AutoTrajectory launchToFuel = routine.trajectory("LaunchToFuel");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                Commands.print("Going to fuel"),
                faceFuelTrench.resetOdometry(),
                faceFuelTrench.cmd()));

    faceFuelTrench
        .done()
        .onTrue(
            Commands.sequence(
                Commands.print("Starting intake and collecting fuel"),
                Commands.parallel(
                    intakeExtension.extendCommand(), intakeRoller.intake(), intakeFuel.cmd())));

    intakeFuel
        .done()
        .onTrue(
            Commands.sequence(
                Commands.print("Moving to shooting position"),
                Commands.sequence(Commands.race(intakeRoller.stop(), new WaitCommand(1))),
                moveToLaunchTrench.cmd()));
    moveToLaunchTrench
        .done()
        .onTrue(
            Commands.sequence(
                Commands.print("Starting launch sequence"),
                Commands.sequence(
                        Commands.race(otfShotSupplier.get(), new WaitCommand(6)),
                        Commands.race(hood.retract(), new WaitCommand(1)),
                        launchToFuel.cmd())
                    .withName("OTF Shooting")));

    launchToFuel
        .done()
        .onTrue(
            Commands.sequence(
                Commands.print("Moving to shooting position"),
                Commands.parallel(
                    intakeExtension.extendCommand(), intakeRoller.intake(), intakeFuel.cmd())));
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
    return NeutralScoreNeutral.getRoutine(
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
