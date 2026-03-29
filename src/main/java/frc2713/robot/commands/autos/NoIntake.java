package frc2713.robot.commands.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc2713.robot.RobotContainer;
import frc2713.robot.subsystems.drive.Drive;
import frc2713.robot.util.AutoUtil;

public class NoIntake {
  public static Command getRoutine(AutoFactory factory, boolean doFlip, Drive driveSubsystem) {
    AutoRoutine routine = factory.newRoutine("NoIntake");

    AutoTrajectory noIntake =
        AutoUtil.flipHorizontalIf(doFlip, routine.trajectory("NoIntake"), routine);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                Commands.print("[AUTO] No Intake"),
                noIntake.resetOdometry(),
                RobotContainer.vision.setGyroAngleCmd(noIntake),
                noIntake.cmd()));

    noIntake.done().onTrue(Commands.run(() -> driveSubsystem.stop()));

    return routine.cmd();
  }
}
