package frc2713.robot.subsystems.launcher;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import frc2713.robot.FieldConstants;
import frc2713.robot.GamePieceConstants;
import frc2713.robot.util.BallTrajectorySim.Ball;
import java.util.ArrayList;

public class FuelTrajectories {
  private ArrayList<Ball> fuel = new ArrayList<>();
  private static Mass fuelMass = GamePieceConstants.Fuel.mass;
  private static Distance fuelRadius = GamePieceConstants.Fuel.radius;
  private static double fuelDragCoeff = GamePieceConstants.Fuel.dragCoeff;
  private static double fuelLiftCoeff = GamePieceConstants.Fuel.liftCoeff;

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
