package frc2713.robot.commands.autos;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc2713.robot.RobotContainer;

public class Demo {
  public static Command demo() {
    return Commands.repeatingSequence(
        Commands.parallel(
                RobotContainer.hood.setAngleCommand(() -> Degrees.of(0.5))
                //   ,  RobotContainer.flywheels.setVelocity(() -> RPM.of(0))
                )
            .withTimeout(0.5),
        RobotContainer.turret.setAngle(() -> Degree.of(-15)).withTimeout(1),
        Commands.parallel(
                RobotContainer.hood.setAngleCommand(() -> Degrees.of(10)),
                RobotContainer.turret.setAngle(() -> Degree.of(15))
                //   ,  RobotContainer.flywheels.setVelocity(() -> RPM.of(400))
                )
            .withTimeout(1),
        RobotContainer.turret.setAngle(() -> Degree.of(0)).withTimeout(1));
  }
}
