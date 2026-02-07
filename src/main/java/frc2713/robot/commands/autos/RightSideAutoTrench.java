package frc2713.robot.commands.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc2713.robot.subsystems.drive.Drive;
import frc2713.robot.subsystems.intake.IntakeExtension;
import frc2713.robot.subsystems.intake.IntakeRoller;
import frc2713.robot.subsystems.launcher.Flywheels;
import frc2713.robot.subsystems.serializer.DyeRotor;
import frc2713.robot.subsystems.serializer.Feeder;

public class RightSideAutoTrench {
  public static AutoRoutine getRoutine(
      AutoFactory factory,
      Drive driveSubsystem,
      IntakeExtension intakeExtension,
      IntakeRoller intakeRoller,
      Flywheels launcher,
      DyeRotor cerealiser,
      //   Launcher intakeAndShooter,
      Feeder feederAndIndexer) {
    AutoRoutine routine = factory.newRoutine("Start Collect Shoot");

    AutoTrajectory faceFuelTrench = routine.trajectory("FaceFuelTrench");
    AutoTrajectory intakeFuel = routine.trajectory("IntakeFuel");
    AutoTrajectory moveToLaunchBump = routine.trajectory("moveToLaunchBump");

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
                Commands.parallel(intakeExtension.extendCommand(), intakeFuel.cmd())));
                
    intakeFuel
        .done()
        .onTrue(
            Commands.sequence(
                Commands.print("Moving to shooting position"),
                // intakeAndShooter.voltageCmd(IntakeAndLauncherConstants.intakeVoltage.get()),
                // new InstantCommand(() -> driveSubsystem.stop()),
                new WaitCommand(2),
                moveToLaunchBump.cmd()));
    moveToLaunchBump
        .done()
        .onTrue(
            Commands.sequence(
                Commands.print("Starting launch sequence"),
                Commands.race(
                    Commands.parallel(
                        // IntakeRoller.voltageCmd(IntakeConstants.extendedPosition.get()),
                        // Turret.voltageCmd(LauncherConstants.launchVoltage.get()),
                        Commands.sequence(
                            // Commands.waitSeconds(IntakeAndLauncherConstants.launchWarmUpTime.get()),
                            Commands.sequence(
                                new InstantCommand(() -> Commands.print("Feeding fuel to launcher"))
                                // feederAndIndexer.voltageCmd(FeederConstants.launchVoltage.get())))),
                                // Commands.waitSeconds(AutoConstants.launchDuration.get()))));
                                ))))));
    return routine;
  }

  public static Command routine(
      AutoFactory factory,
      Drive driveSubsystem,
      IntakeExtension intakeExtension,
      IntakeRoller intakeRoller,
      Flywheels launcher,
      DyeRotor cerealiser,
      Feeder feederAndIndexer) {
    return RightSideAutoTrench.getRoutine(
            factory,
            driveSubsystem,
            intakeExtension,
            intakeRoller,
            launcher,
            cerealiser,
            feederAndIndexer)
        .cmd();
  }
}
