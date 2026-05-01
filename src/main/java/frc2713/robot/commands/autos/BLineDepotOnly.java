package frc2713.robot.commands.autos;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.Path;
import frc2713.lib.util.AllianceFlipUtil;
import frc2713.lib.util.WaitSupplierCommand;
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
        new WaitSupplierCommand(() -> SmartDashboard.getNumber("autoStartDelay", 0)),
        // Shoot preloaded fuel and wait
        Commands.runOnce(
            () ->
                RobotContainer.drive.setPose(AllianceFlipUtil.apply(startingPath.getStartPose()))),

        // Drive to Depot while intaking
        RobotContainer.pathBuilder
            .withPoseReset(RobotContainer.drive::setPose)
            // .withEvent(
            //     "intake",
            //     GameCommandGroups.Intaking.intake(
            //         RobotContainer.intakeExtension, RobotContainer.intakeRoller))
            .build(startingPath),
        GameCommandGroups.Intaking.intake(
                RobotContainer.intakeExtension, RobotContainer.intakeRoller)
            .withTimeout(1),
        // Drive to launch position
        RobotContainer.pathBuilder
            .withPoseReset(pose -> {})
            .withStartingEvent(RobotContainer.intakeRoller.intake())
            .withEvent(
                "intake_2",
                GameCommandGroups.Intaking.intake(
                    RobotContainer.intakeExtension, RobotContainer.intakeRoller))
            .build(new Path("depot_only_second"))
            .andThen(
                Commands.sequence(
                    GameCommandGroups.Launching.autoOtfShot(
                            RobotContainer.drive,
                            RobotContainer.flywheels,
                            RobotContainer.hood,
                            RobotContainer.turret,
                            RobotContainer.feeder,
                            RobotContainer.dyeRotor,
                            RobotContainer.intakeExtension,
                            RobotContainer.intakeRoller)
                        .withDeadline(Commands.waitSeconds(wait2.in(Seconds))),
                    GameCommandGroups.Launching.stopShootingAndRetractHood(
                            RobotContainer.drive,
                            RobotContainer.feeder,
                            RobotContainer.dyeRotor,
                            RobotContainer.hood,
                            RobotContainer.flywheels)
                        .withTimeout(0.25))),
        // back to midline
        RobotContainer.pathBuilder
            .withPoseReset(pose -> {})
            .withEvent(
                "intake",
                GameCommandGroups.Intaking.intake(
                    RobotContainer.intakeExtension, RobotContainer.intakeRoller))
            .build(new Path("depot_only_third")),
        Commands.run(() -> RobotContainer.drive.stop()));
  }
}
