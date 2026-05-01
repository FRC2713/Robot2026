package frc2713.robot.commands.autos;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.Path;
import frc2713.lib.util.AllianceFlipUtil;
import frc2713.robot.GameCommandGroups;
import frc2713.robot.RobotContainer;

public class BLineDepotOnly {

  // Note: starting position may be different than midwars, see path

  // For reference, with no delays these are timings from our SIM test
  // From start -> depot: ~1.5s
  // From depot -> trench entrance: ~2.5s
  // From trench entrance -> midline: 2.0s

  private static Time wait1 = Seconds.of(0.5); // before path starts
  private static Time wait2 = Seconds.of(7.0); // shooting duration before going through trench
  private static Path startingPath = new Path("depot_only");

  public static Command getCommand() {

    return Commands.sequence(
        // Shoot preloaded fuel and wait
        Commands.runOnce(
            () ->
                RobotContainer.drive.setPose(AllianceFlipUtil.apply(startingPath.getStartPose()))),

        // Drive to Depot while intaking
        RobotContainer.pathBuilder
            .withPoseReset(RobotContainer.drive::setPose)
            .withEvent(
                "intake",
                GameCommandGroups.Intaking.intake(
                    RobotContainer.intakeExtension, RobotContainer.intakeRoller))
            .build(startingPath),
        // Drive to launch position
        RobotContainer.pathBuilder
            .withPoseReset(pose -> {})
            .withStartingEvent(
                GameCommandGroups.Launching.stopShootingAndRetractHood(
                    RobotContainer.drive,
                    RobotContainer.feeder,
                    RobotContainer.dyeRotor,
                    RobotContainer.hood,
                    RobotContainer.flywheels))
            .withEvent(
                "intake_2",
                Commands.parallel(
                    RobotContainer.hood.retract(),
                    GameCommandGroups.Intaking.intake(
                        RobotContainer.intakeExtension, RobotContainer.intakeRoller)))
            .build(new Path("depot_only_second"))
            .andThen(
                GameCommandGroups.Launching.autoOtfShot(
                        RobotContainer.drive,
                        RobotContainer.flywheels,
                        RobotContainer.hood,
                        RobotContainer.turret,
                        RobotContainer.feeder,
                        RobotContainer.dyeRotor,
                        RobotContainer.intakeExtension,
                        RobotContainer.intakeRoller)
                    .withDeadline(Commands.waitSeconds(wait2.in(Seconds))).andThen(
                        GameCommandGroups.Launching.stopShooting(RobotContainer.drive, RobotContainer.feeder, RobotContainer.dyeRotor, RobotContainer.flywheels).withTimeout(0.25)
                    )),
        Commands.run(() -> RobotContainer.drive.stop()));
  }
}
