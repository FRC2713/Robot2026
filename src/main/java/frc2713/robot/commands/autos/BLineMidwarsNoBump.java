package frc2713.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.*;
import frc2713.robot.GameCommandGroups;
import frc2713.robot.RobotContainer;
import java.util.function.Supplier;

public class BLineMidwarsNoBump {

  public static Command getCommand(Supplier<Boolean> shouldMirror) {
    return Commands.sequence(
        // Drive through neutral zone with intake and shoot events
        RobotContainer.pathBuilder
            .withPoseReset(RobotContainer.drive::setPose)
            .withShouldMirror(shouldMirror)
            .withEvent(
                "intake",
                GameCommandGroups.Intaking.intake(
                    RobotContainer.intakeExtension, RobotContainer.intakeRoller))
            .withEvent(
                "shoot_1",
                GameCommandGroups.Launching.autoOtfShot(
                    RobotContainer.drive,
                    RobotContainer.flywheels,
                    RobotContainer.hood,
                    RobotContainer.turret,
                    RobotContainer.feeder,
                    RobotContainer.dyeRotor,
                    RobotContainer.intakeExtension,
                    RobotContainer.intakeRoller))
            .build(new Path("nobump_mid_wards")),
        // Pausing to shoot
        GameCommandGroups.Launching.autoOtfShot(
                RobotContainer.drive,
                RobotContainer.flywheels,
                RobotContainer.hood,
                RobotContainer.turret,
                RobotContainer.feeder,
                RobotContainer.dyeRotor,
                RobotContainer.intakeExtension,
                RobotContainer.intakeRoller)
            .withDeadline(Commands.waitSeconds(2)),

        // Drive through neutral zone with intake and shoot events
        RobotContainer.pathBuilder
            .withShouldMirror(shouldMirror)
            .withPoseReset(pose -> {})
            .withStartingEvent(
                Commands.parallel(
                    GameCommandGroups.Launching.stopShootingAndRetractHood(
                        RobotContainer.drive,
                        RobotContainer.feeder,
                        RobotContainer.dyeRotor,
                        RobotContainer.hood,
                        RobotContainer.flywheels),
                    GameCommandGroups.Intaking.intake(
                        RobotContainer.intakeExtension, RobotContainer.intakeRoller)))
            .withEvent(
                "shoot_2",
                GameCommandGroups.Launching.autoOtfShot(
                        RobotContainer.drive,
                        RobotContainer.flywheels,
                        RobotContainer.hood,
                        RobotContainer.turret,
                        RobotContainer.feeder,
                        RobotContainer.dyeRotor,
                        RobotContainer.intakeExtension,
                        RobotContainer.intakeRoller)
                    .repeatedly())
            .build(new Path("nobump_mid_wards_straight")),
        // Shooting
        GameCommandGroups.Launching.autoOtfShot(
                RobotContainer.drive,
                RobotContainer.flywheels,
                RobotContainer.hood,
                RobotContainer.turret,
                RobotContainer.feeder,
                RobotContainer.dyeRotor,
                RobotContainer.intakeExtension,
                RobotContainer.intakeRoller)
            .repeatedly());
  }
}
