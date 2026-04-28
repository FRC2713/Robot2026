package frc2713.robot.commands.autos;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.Path;
import frc2713.robot.GameCommandGroups;
import frc2713.robot.RobotContainer;

public class BLineSweepAndOutpost {

  // Note: starting position may be different than midwars, see path

  // For reference, with no delays these are timings from our IRL test
  // From start -> over the bump: ~6s
  // From over the bump ->  outpost: ~4s

  private static Time wait1 = Seconds.of(0.0); // before routine starts
  private static Time wait2 = Seconds.of(0.5); // after going over bump
  private static Time fuelPressureDelay = Seconds.of(5.0); // after getting to outpost

  public static Command getCommand() {
    return Commands.sequence(
        Commands.waitSeconds(wait1.in(Seconds)),
        // Complete first sweep close to hub and shoot after bump
        RobotContainer.pathBuilder
            .withPoseReset(RobotContainer.drive::setPose)
            .withShouldMirror(() -> false)
            .withEvent(
                "intake",
                GameCommandGroups.Intaking.intake(
                    RobotContainer.intakeExtension, RobotContainer.intakeRoller))
            .withEvent(
                "shoot",
                GameCommandGroups.Launching.autoOtfShotNoPressure(
                    RobotContainer.drive,
                    RobotContainer.flywheels,
                    RobotContainer.hood,
                    RobotContainer.turret,
                    RobotContainer.feeder,
                    RobotContainer.dyeRotor,
                    RobotContainer.intakeExtension,
                    RobotContainer.intakeRoller))
            .build(new Path("sweep_n_outpost")),
        // Pausing after bump to shoot
        GameCommandGroups.Launching.autoOtfShotNoPressure(
                RobotContainer.drive,
                RobotContainer.flywheels,
                RobotContainer.hood,
                RobotContainer.turret,
                RobotContainer.feeder,
                RobotContainer.dyeRotor,
                RobotContainer.intakeExtension,
                RobotContainer.intakeRoller)
            .withDeadline(Commands.waitSeconds(wait2.in(Seconds))),
        // Drive to trench while shooting, and then put intake out
        RobotContainer.pathBuilder
            .withPoseReset(pose -> {})
            .withShouldMirror(() -> false)
            .withEvent(
                "shoot_2",
                GameCommandGroups.Launching.autoOtfShotNoPressure(
                    RobotContainer.drive,
                    RobotContainer.flywheels,
                    RobotContainer.hood,
                    RobotContainer.turret,
                    RobotContainer.feeder,
                    RobotContainer.dyeRotor,
                    RobotContainer.intakeExtension,
                    RobotContainer.intakeRoller))
            .withEvent(
                "intake_2",
                GameCommandGroups.Intaking.intake(
                    RobotContainer.intakeExtension, RobotContainer.intakeRoller))
            .build(new Path("sweep_n_outpost_second")),
        Commands.parallel(
            GameCommandGroups.Launching.autoOtfShot(
                RobotContainer.drive,
                RobotContainer.flywheels,
                RobotContainer.hood,
                RobotContainer.turret,
                RobotContainer.feeder,
                RobotContainer.dyeRotor,
                RobotContainer.intakeExtension,
                RobotContainer.intakeRoller,
                () -> fuelPressureDelay.in(Seconds)),
            Commands.run(() -> RobotContainer.drive.stop())));
  }
}
