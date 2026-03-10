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
    AutoRoutine routine = factory.newRoutine("NeutralScoreNeutral");

    AutoTrajectory faceFuelTrench = routine.trajectory("FaceFuelTrench");
    AutoTrajectory intakeFuel = routine.trajectory("IntakeFuel");
    AutoTrajectory moveToLaunchTrench = routine.trajectory("MoveToLaunchTrench");
    AutoTrajectory launchToFuel = routine.trajectory("LaunchToFuel");
    AutoTrajectory intakeFuel2 = routine.trajectory("IntakeFuel");

    faceFuelTrench
        .atTranslation("BeginIntaking", 0.2)
        .onTrue(
            Commands.parallel(
                Commands.print("[AUTO] Marker-based intake start"),
                intakeExtension.extendCommand(),
                intakeRoller.intake()));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                Commands.print("[AUTO] Going to fuel"),
                faceFuelTrench.resetOdometry(),
                faceFuelTrench.cmd()));

    faceFuelTrench
        .done()
        .onTrue(
            Commands.parallel(
                Commands.print("[AUTO] Starting intake and collecting fuel"),
                intakeExtension.extendCommand(),
                intakeRoller.intake(),
                intakeFuel.cmd()));

    intakeFuel
        .done()
        .onTrue(
            Commands.sequence(
                Commands.print("[AUTO] Moving to shooting position"), moveToLaunchTrench.cmd()));

    moveToLaunchTrench
        .done()
        .onTrue(
            Commands.sequence(
                Commands.print("[AUTO] Starting launch sequence"),
                Commands.sequence(
                        Commands.race(otfShotSupplier.get(), new WaitCommand(6)),
                        Commands.sequence(
                            hood.retract().withTimeout(0.1),
                            Commands.parallel(
                                launchToFuel.cmd(),
                                GameCommandGroups.Launching.stopShooting(
                                    driveSubsystem, feeder, dyeRotor))))
                    .withName("OTF Shooting")));

    launchToFuel
        .done()
        .onTrue(
            Commands.sequence(
                Commands.print("[AUTO] Going to fuel again"),
                Commands.parallel(
                    intakeExtension.extendCommand(),
                    intakeRoller.intake(),
                    intakeFuel2.cmd(),
                    Commands.run(() -> driveSubsystem.stop()))));
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
