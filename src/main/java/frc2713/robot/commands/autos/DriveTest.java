package frc2713.robot.commands.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveTest {
  public static AutoRoutine getRoutine(AutoFactory factory) {
    AutoRoutine routine = factory.newRoutine("DriveTest");

    AutoTrajectory driveTest = routine.trajectory("DriveTest");

    routine.active().onTrue(driveTest.cmd());

    return routine;
  }

  public static Command routine(AutoFactory factory) {
    return DriveTest.getRoutine(factory).cmd();
  }
}
