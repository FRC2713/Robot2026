package frc2713.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.Path;
import frc2713.robot.GameCommandGroups;
import frc2713.robot.RobotContainer;

public class BLineDepotOnly {

  public static Command getCommand() {
    return Commands.sequence(
        // Shoot preloaded fuel and wait
        GameCommandGroups.Launching.autoOtfShot(
                RobotContainer.drive,
                RobotContainer.flywheels,
                RobotContainer.hood,
                RobotContainer.turret,
                RobotContainer.feeder,
                RobotContainer.dyeRotor,
                RobotContainer.intakeExtension,
                RobotContainer.intakeRoller)
            .withTimeout(1.0),
        // Drive to Depot while intaking and prep shot
        RobotContainer.pathBuilder
            .withPoseReset(RobotContainer.drive::setPose)
            .withEvent(
                "intake",
                GameCommandGroups.Intaking.intake(
                    RobotContainer.intakeExtension, RobotContainer.intakeRoller))
            .withEvent(
                "shoot",
                GameCommandGroups.Launching.autoOtfShot(
                    RobotContainer.drive,
                    RobotContainer.flywheels,
                    RobotContainer.hood,
                    RobotContainer.turret,
                    RobotContainer.feeder,
                    RobotContainer.dyeRotor,
                    RobotContainer.intakeExtension,
                    RobotContainer.intakeRoller))
            .build(new Path("depot_only")),
        // Pausing near depot to shoot
        GameCommandGroups.Launching.autoOtfShot(
                RobotContainer.drive,
                RobotContainer.flywheels,
                RobotContainer.hood,
                RobotContainer.turret,
                RobotContainer.feeder,
                RobotContainer.dyeRotor,
                RobotContainer.intakeExtension,
                RobotContainer.intakeRoller)
            .withDeadline(Commands.waitSeconds(5.0)),
        // Drive to midline
        RobotContainer.pathBuilder
            .withPoseReset(pose -> {})
            .withStartingEvent(
                GameCommandGroups.Launching.autoOtfShot(
                    RobotContainer.drive,
                    RobotContainer.flywheels,
                    RobotContainer.hood,
                    RobotContainer.turret,
                    RobotContainer.feeder,
                    RobotContainer.dyeRotor,
                    RobotContainer.intakeExtension,
                    RobotContainer.intakeRoller))
            .build(new Path("depot_only_second")),
        Commands.run(() -> RobotContainer.drive.stop()));
  }
}
