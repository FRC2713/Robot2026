package frc2713.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.*;
import frc2713.robot.RobotContainer;

public class BLineTuning {

  // Load Paths
  static Path path = new Path("2m_straight");

  public static Command getCommand() {
    return Commands.sequence(RobotContainer.pathBuilder.build(path));
  }
}
