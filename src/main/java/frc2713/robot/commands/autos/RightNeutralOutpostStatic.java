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

/**
 * Starts at right trench. Collects from Neutral Zone once. Goes to RIGHT trench. Shoots fo X
 * seconds Goes to Outpost. Shoots at Outpost for Y seconds. Moves to Neutral Zone.
 */
public class RightNeutralOutpostStatic {
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
    AutoRoutine routine = factory.newRoutine("RightNeutralOutpostStatic");

    AutoTrajectory intakeFuelRight = routine.trajectory("IntakeFuelRight");
    AutoTrajectory neutralToRightTrenchForward = routine.trajectory("NeutralToRightTrenchForward");
    AutoTrajectory oTFToOutpost = routine.trajectory("OTFToOutpost");
    AutoTrajectory outpostToTrench = routine.trajectory("OutpostToTrench");
    AutoTrajectory faceFuelRightTrenchBackward = routine.trajectory("FaceFuelRightTrenchBackward");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                Commands.print("[AUTO] Going to fuel"),
                intakeFuelRight.resetOdometry(),
                // otfShotSupplier.get().withTimeout(1),
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
                Commands.print("[AUTO] Moving to trench"),
                Commands.runOnce(() -> driveSubsystem.stop()),
                Commands.race(intakeRoller.stop(), new WaitCommand(0.2)),
                neutralToRightTrenchForward.cmd()));

    neutralToRightTrenchForward
        .done()
        .onTrue(
            Commands.sequence(
                Commands.print("[AUTO] Shooting from trench"),
                Commands.runOnce(() -> driveSubsystem.stop()),
                Commands.deadline(Commands.waitSeconds(5.0), otfShotSupplier.get()),
                oTFToOutpost.cmd()));

    oTFToOutpost
        .done()
        .onTrue(
            Commands.deadline(
                Commands.sequence(
                    Commands.runOnce(() -> driveSubsystem.stop()),
                    Commands.print("[AUTO] Launching at outpost"),
                    Commands.deadline(Commands.waitSeconds(3), otfShotSupplier.get()))));

    outpostToTrench.done().onTrue(faceFuelRightTrenchBackward.cmd());

    faceFuelRightTrenchBackward.done().onTrue(Commands.runOnce(() -> driveSubsystem.stop()));

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
    return RightNeutralOutpostStatic.getRoutine(
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
