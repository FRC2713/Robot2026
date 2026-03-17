package frc2713.robot.util;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc2713.robot.FieldConstants;
import java.util.ArrayList;

public class AutoUtil {
  /**
   * Returns this trajectory, mirrored across the HORIZONTAL field midline.
   *
   * @param trajectory the trajectory to mirror.
   * @return trajectory, mirrored across the HORIZONTAL field midline.
   */
  public static AutoTrajectory flipHorizontal(AutoTrajectory autotrajectory, AutoRoutine routine) {
    Trajectory<SwerveSample> trajectory = autotrajectory.getRawTrajectory();
    var flippedStates = new ArrayList<SwerveSample>();
    for (var state : trajectory.samples()) {
      flippedStates.add(
          new SwerveSample(
              state.t,
              state.x,
              FieldConstants.fieldWidth - (state.y),
              -state.heading,
              state.vx,
              -state.vy,
              -state.omega,
              state.ax,
              -state.ay,
              -state.alpha,
              // TODO: VERIFY THIS
              // FL, FR, BL, BR
              // Mirrored
              // FR, FL, BR, BL
              new double[] {
                state.moduleForcesX()[1],
                state.moduleForcesX()[0],
                state.moduleForcesX()[3],
                state.moduleForcesX()[2]
              },
              // FL, FR, BL, BR
              // Mirrored
              // -FR, -FL, -BR, -BL
              new double[] {
                -state.moduleForcesY()[1],
                -state.moduleForcesY()[0],
                -state.moduleForcesY()[3],
                -state.moduleForcesY()[2]
              }));
    }
    var newtraj =
        new Trajectory<SwerveSample>(
            trajectory.name(), flippedStates, trajectory.splits(), trajectory.events());
    return routine.trajectory(newtraj);
  }
}
