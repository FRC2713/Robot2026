package frc2713.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.*;
import frc2713.robot.GameCommandGroups;
import frc2713.robot.RobotContainer;

public class BLineMidwarsOvercenter {

  // Load Paths
  static Path midWars = new Path("mid_wards_overcenter");
  static Path backToTrench = new Path("back_to_trench");
  static Path midWarsStraight = new Path("mid_wards_straight");

  public static Command getCommand() {
    return Commands.sequence(
        GameCommandGroups.Intaking.intake(
                RobotContainer.intakeExtension, RobotContainer.intakeRoller)
            .withDeadline(RobotContainer.pathBuilder.build(midWars)),
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
            RobotContainer.pathBuilder.build(backToTrench)),
        GameCommandGroups.Intaking.intake(
                RobotContainer.intakeExtension, RobotContainer.intakeRoller)
            .withDeadline(RobotContainer.pathBuilder.build(midWarsStraight)),
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
            RobotContainer.pathBuilder.build(backToTrench)));
  }
}
