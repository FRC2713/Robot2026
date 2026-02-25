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

public class NeutralSweepLeftToRight {
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
    AutoRoutine routine = factory.newRoutine("Start Neutral Sweep Left to Right");

    AutoTrajectory faceFuelTrenchLeft = routine.trajectory("FaceFuelTrenchLeft");
    AutoTrajectory intakeFuelLeft = routine.trajectory("IntakeFuelLeft");
    AutoTrajectory sweepToLaunchTrenchRight = routine.trajectory("SweepToLaunchTrenchRight");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                Commands.print("Going to fuel"),
                faceFuelTrenchLeft.resetOdometry(),
                faceFuelTrenchLeft.cmd()));

    faceFuelTrenchLeft
        .done()
        .onTrue(
            Commands.sequence(
                Commands.print("Starting intake and collecting fuel"),
                Commands.parallel(
                    intakeExtension.extendCommand(), intakeRoller.intake(), intakeFuelLeft.cmd())));

    intakeFuelLeft
        .done()
        .onTrue(
            Commands.sequence(
                Commands.print("Sweeping to launching position"),
                Commands.sequence(Commands.race(intakeRoller.stop(), new WaitCommand(0.5))),
                sweepToLaunchTrenchRight.cmd()));

    sweepToLaunchTrenchRight.done().onTrue(otfShotSupplier.get());

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
    return NeutralSweepLeftToRight.getRoutine(
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
