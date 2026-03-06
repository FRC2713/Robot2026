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

public class RightNeutralOutpost {
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

    AutoTrajectory intakeFuelRight = routine.trajectory("IntakeFuelRight");
    AutoTrajectory neutralToRightTrenchForward = routine.trajectory("NeutralToRightTrenchForward");
    AutoTrajectory oTFToOutpost = routine.trajectory("OTFToOutpost");
    AutoTrajectory outpostToTrench = routine.trajectory("OutpostToTrench");
    AutoTrajectory faceFuelRightTrenchBackward = routine.trajectory("FaceFuelRightTrenchBackward");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                Commands.print("Going to fuel"),
                intakeFuelRight.resetOdometry(),
                Commands.parallel(
                    intakeFuelRight.cmd(),
                    Commands.sequence(
                        new WaitCommand(0.3),
                        intakeExtension.extendCommand(),
                        intakeRoller.intake()))));

    intakeFuelRight
        .done()
        .onTrue(
            Commands.sequence(
                Commands.print("Moving to trench"),
                Commands.race(intakeRoller.stop(), new WaitCommand(0.2)),
                neutralToRightTrenchForward.cmd()));

    neutralToRightTrenchForward
        .done()
        .onTrue(Commands.parallel(otfShotSupplier.get(), oTFToOutpost.cmd()));

    // oTFToOutpost
    //     .done()
    //     .onTrue(
    //         Commands.deadline(
    //             Commands.sequence(
    //                 Commands.print("Launching at outpost"),
    //                 Commands.waitSeconds(4),
    //                 Commands.print("Moving back to trench"),
    //                 outpostToTrench.cmd()),
    //             otfShotSupplier.get()));

    outpostToTrench.done().onTrue(faceFuelRightTrenchBackward.cmd());

    faceFuelRightTrenchBackward.done();

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
    return RightNeutralOutpost.getRoutine(
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
