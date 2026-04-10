package frc2713.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.Path;
import frc2713.robot.GameCommandGroups;
import frc2713.robot.RobotContainer;
import java.util.function.Supplier;

public class BLineMidwarsOvercenter {

  public static Command getCommand(Supplier<Boolean> shouldMirror) {
    return Commands.sequence(
        GameCommandGroups.Intaking.intake(
                RobotContainer.intakeExtension, RobotContainer.intakeRoller)
            .withDeadline(
                RobotContainer.pathBuilder
                    .withPoseReset(RobotContainer.drive::setPose)
                    .withShouldMirror(shouldMirror)
                    .build(new Path("mid_wards_overcenter"))),
        Commands.parallel(
            GameCommandGroups.Launching.otfShot(
                    RobotContainer.drive,
                    RobotContainer.flywheels,
                    RobotContainer.hood,
                    RobotContainer.turret,
                    RobotContainer.feeder,
                    RobotContainer.dyeRotor,
                    RobotContainer.intakeExtension,
                    RobotContainer.intakeRoller)
                .withDeadline(Commands.waitSeconds(5.0)),
            RobotContainer.pathBuilder
                .withPoseReset(pose -> {})
                .withShouldMirror(shouldMirror)
                .build(new Path("back_to_trench"))),
        GameCommandGroups.Intaking.intake(
                RobotContainer.intakeExtension, RobotContainer.intakeRoller)
            .withDeadline(
                RobotContainer.pathBuilder
                    .withShouldMirror(shouldMirror)
                    .withPoseReset(pose -> {})
                    .build(new Path("mid_wards_straight"))),
        Commands.parallel(
            GameCommandGroups.Launching.otfShot(
                RobotContainer.drive,
                RobotContainer.flywheels,
                RobotContainer.hood,
                RobotContainer.turret,
                RobotContainer.feeder,
                RobotContainer.dyeRotor,
                RobotContainer.intakeExtension,
                RobotContainer.intakeRoller),
            RobotContainer.pathBuilder
                .withShouldMirror(shouldMirror)
                .withPoseReset(pose -> {})
                .build(new Path("back_to_trench"))));
  }
}
