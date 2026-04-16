package frc2713.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.Path;
import frc2713.robot.GameCommandGroups;
import frc2713.robot.RobotContainer;
import java.util.function.Supplier;

public class BLineMidwarsConservative {

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
            .build(new Path("mid_wards_conservative")),
        // Pausing near hub to shoot
        GameCommandGroups.Launching.autoOtfShot(
                RobotContainer.drive,
                RobotContainer.flywheels,
                RobotContainer.hood,
                RobotContainer.turret,
                RobotContainer.feeder,
                RobotContainer.dyeRotor,
                RobotContainer.intakeExtension,
                RobotContainer.intakeRoller)
            .withDeadline(Commands.waitSeconds(0.25)),
        // Drive to trench while shooting
        RobotContainer.pathBuilder
            .withPoseReset(pose -> {})
            .withShouldMirror(shouldMirror)
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
            .build(new Path("back_to_trench")),
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
            .build(new Path("mid_wards_conservative_second")),
        // Drive to trench while shooting
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
