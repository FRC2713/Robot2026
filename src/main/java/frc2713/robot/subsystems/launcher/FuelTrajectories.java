package frc2713.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import frc2713.robot.FieldConstants;
import frc2713.robot.util.BallTrajectorySim.Ball;
import java.util.ArrayList;

public class FuelTrajectories {
  private ArrayList<Ball> fuel = new ArrayList<>();
  private static Mass fuelMass = Pounds.of(0.5);
  private static Distance fuelRadius = Inches.of(5.91 / 2);
  private static double fuelDragCoeff = 0.47;
  private static double fuelLiftCoeff = 0.2;

  public FuelTrajectories() {}

  public void launch(Translation3d initialPose, Translation3d initialVel, AngularVelocity spin) {
    Ball newBall = new Ball(fuelMass, fuelRadius, fuelDragCoeff, fuelLiftCoeff);
    newBall.launch(initialPose, initialVel, spin);
    this.fuel.add(newBall);
  }

  public void update(Time dt) {
    fuel.removeIf(
        ball -> {
          ball.update(dt);
          return ball.getPosition().getZ() < FieldConstants.Hub.innerHeight
              && ball.getVelocity().getZ() < 0;
        });
  }

  public Translation3d[] getPositions() {
    Translation3d[] t = new Translation3d[fuel.size()];
    for (int i = 0; i < fuel.size(); i++) {
      t[i] = fuel.get(i).position;
    }
    return t;
  }
}
