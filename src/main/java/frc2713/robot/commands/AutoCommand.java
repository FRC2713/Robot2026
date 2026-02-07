package frc2713.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc2713.robot.subsystems.intake.IntakeExtension;

public class AutoCommand {
  public static AutoRoutine getRoutine(
      AutoFactory factory,
      DriveState driveSubsystem,
      IntakeExtension IntakeRoller,
      //   Launcher intakeAndShooter,
      frc2713.robot.subsystems.serializer.Feeder feederAndIndexer) {
    AutoRoutine routine = factory.newRoutine("Start Collect Shoot");

    AutoTrajectory faceFuelTrench = routine.trajectory("FaceFuel");
    AutoTrajectory IntakeFuel = routine.trajectory("CollectFuel");
    AutoTrajectory MoveToLaunchTrench = routine.trajectory("FuelToShotT");

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
                    // intakeAndShooter.voltageCmd(IntakeAndLauncherConstants.intakeVoltage.get()),
                    IntakeFuel.cmd())));
    IntakeFuel.done()
        .onTrue(
            Commands.sequence(
                Commands.print("Moving to shooting position"),
                // intakeAndShooter.voltageCmd(IntakeAndLauncherConstants.intakeVoltage.get()),
                // new InstantCommand(() -> driveSubsystem.stop()),
                new WaitCommand(2),
                MoveToLaunchTrench.cmd()));
    MoveToLaunchTrench.done()
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
}
