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

public class RightSideAutoTrench {
  public static AutoRoutine getRoutine(
      AutoFactory factory,
      Drive driveSubsystem,
      IntakeExtension intakeExtension,
      IntakeRoller intakeRoller,
      Flywheels flywheels,
      Hood hood,
      Turret turret,
      DyeRotor serializer,
      //   Launcher intakeAndShooter,
      Feeder feederAndIndexer) {
    AutoRoutine routine = factory.newRoutine("Start Collect Shoot");

    AutoTrajectory faceFuelTrench = routine.trajectory("FaceFuelTrench");
    AutoTrajectory intakeFuel = routine.trajectory("IntakeFuel");
    AutoTrajectory moveToLaunchBump = routine.trajectory("MoveToLaunchBump");

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
                Commands.sequence(Commands.race(intakeRoller.stop(), new WaitCommand(2))),
                moveToLaunchBump.cmd()));
    moveToLaunchBump
        .done()
        .onTrue(
            Commands.sequence(
                Commands.print("Starting launch sequence"),
                Commands.race(
                        // hub shot
                        //     flywheels.hubCommand(),
                        //     hood.hubCommand(),
                        //     turret.hubCommand(),
                        //     flywheels.simulateLaunchedFuel(
                        //         () -> {
                        //           return flywheels.atTarget() && hood.atTarget() &&
                        // turret.atTarget();
                        //         }),
                        //     feederAndIndexer.feedWhenReady(
                        //         () -> {
                        //           return flywheels.atTarget() && hood.atTarget() &&
                        // turret.atTarget();
                        //         }),
                        //     serializer.feedWhenReady(
                        //         () -> {
                        //           return flywheels.atTarget() && hood.atTarget() &&
                        // turret.atTarget();
                        //         }))
                        // .withName("Hub Shot")));
                        // otf shot
                        flywheels.otfCommand(),
                        hood.otfCommand(),
                        turret.oftCommand(),
                        flywheels.simulateLaunchedFuel(
                            () -> {
                              return flywheels.atTarget() && hood.atTarget() && turret.atTarget();
                            }),
                        feederAndIndexer.feedWhenReady(
                            () -> {
                              return flywheels.atTarget() && hood.atTarget() && turret.atTarget();
                            }),
                        serializer.feedWhenReady(
                            () -> {
                              return flywheels.atTarget() && hood.atTarget() && turret.atTarget();
                            }))
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
      DyeRotor serializer,
      Feeder feederAndIndexer) {
    return RightSideAutoTrench.getRoutine(
            factory,
            driveSubsystem,
            intakeExtension,
            intakeRoller,
            flywheels,
            hood,
            turret,
            serializer,
            feederAndIndexer)
        .cmd();
  }
}
