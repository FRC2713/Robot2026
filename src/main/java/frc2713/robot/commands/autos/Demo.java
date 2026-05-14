package frc2713.robot.commands.autos;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc2713.robot.RobotContainer;
import frc2713.robot.subsystems.intake.IntakeConstants;

public class Demo {
  public static Command demo() {
    return Commands.parallel(
        RobotContainer.turret.setAngle(
            () -> Degree.of(Math.sin(Timer.getFPGATimestamp() / 1.5) * 180)),
        RobotContainer.hood.setAngleCommand(
            () -> Degree.of(Math.max(0.5, Math.sin(Timer.getFPGATimestamp()) * 30))),
        RobotContainer.intakeExtension.setDistanceCommand(
            () ->
                Inches.of(
                    Math.max(
                        1,
                        Math.sin(Timer.getFPGATimestamp())
                                * IntakeConstants.Extension.extendedPosition.get().in(Inches)
                            - 2)),
            IntakeConstants.Extension.retractCruiseVelocity)
        // Commands.repeatingSequence(
        //   RobotContainer.intakeExtension.extendCommand()
        // )
        );
    // Commands.repeatingSequence(
    //     Commands.parallel(
    //             RobotContainer.hood.setAngleCommand(() -> Degrees.of(0.5)),
    //             RobotContainer.flywheels.setVelocity(() -> RPM.of(0)))
    //         .withTimeout(0.5),
    //     // .withTimeout(1),
    //     Commands.parallel(
    //             RobotContainer.hood.setAngleCommand(() -> Degrees.of(10)),
    //             // RobotContainer.turret.setAngle(() -> Degree.of(15)),
    //             RobotContainer.flywheels.setVelocity(() -> RPM.of(400)))
    //         .withTimeout(1)));
    // RobotContainer.turret.setAngle(() -> Degree.of(0)).withTimeout(1));
  }
}
