package frc2713.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.Path;
import frc2713.robot.RobotContainer;

public class BumpTest {

  public static Command getCommand() {
    return Commands.sequence(
        RobotContainer.pathBuilder
            .withPoseReset(RobotContainer.drive::setPose)
            .build(new Path("bumpTest")));
  }
}
