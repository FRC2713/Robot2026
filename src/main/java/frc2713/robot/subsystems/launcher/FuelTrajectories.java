package frc2713.robot.subsystems.launcher;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import frc2713.lib.field.CircularFieldRegion;
import frc2713.robot.FieldConstants;
import frc2713.robot.GamePieceConstants;
import frc2713.robot.util.BallTrajectorySim.Ball;
import frc2713.robot.util.BallTrajectorySim.Ball.TargetResult;
import java.util.ArrayList;

public class FuelTrajectories {
  private static final int MAX_COMPLETED_FLIGHT_TIMES = 10;
  private ArrayList<Ball> fuel = new ArrayList<>();
  private ArrayList<Double> completedFlightTimesSeconds = new ArrayList<>();
  private ArrayList<Double> completedTargetHits = new ArrayList<>();
  private static Mass fuelMass = GamePieceConstants.Fuel.mass;
  private static Distance fuelRadius = GamePieceConstants.Fuel.radius;
  private static double fuelDragCoeff = GamePieceConstants.Fuel.dragCoeff;
  private static double fuelLiftCoeff = GamePieceConstants.Fuel.liftCoeff;

  public FuelTrajectories() {}

  public void launch(Translation3d initialPose, Translation3d initialVel, AngularVelocity spin) {
    launch(initialPose, initialVel, spin, null);
  }

  public void launch(
      Translation3d initialPose,
      Translation3d initialVel,
      AngularVelocity spin,
      CircularFieldRegion targetRegion) {
    Ball newBall = new Ball(fuelMass, fuelRadius, fuelDragCoeff, fuelLiftCoeff);
    newBall.launch(initialPose, initialVel, spin, targetRegion);
    this.fuel.add(newBall);
  }

  public void update(Time dt) {
    fuel.removeIf(
        ball -> {
          ball.update(dt);
          boolean shouldRemove = false;
          if (ball.hasTargetRegion()) {
            CircularFieldRegion targetRegion = ball.getTargetRegion();
            if (ball.getVelocity().getZ() < 0
                && ball.getPosition().getZ() <= targetRegion.getZMeters()) {
              boolean hit =
                  targetRegion.contains(
                      new edu.wpi.first.math.geometry.Translation2d(
                          ball.getPosition().getX(), ball.getPosition().getY()));
              ball.setTargetResult(hit ? TargetResult.HIT : TargetResult.MISS);
              completedTargetHits.add(hit ? 1.0 : 0.0);
              if (completedTargetHits.size() > MAX_COMPLETED_FLIGHT_TIMES) {
                completedTargetHits.remove(0);
              }
              shouldRemove = true;
            }
          } else {
            shouldRemove =
                ball.getPosition().getZ() < FieldConstants.Hub.innerHeight
                    && ball.getVelocity().getZ() < 0;
          }
          if (shouldRemove) {
            completedFlightTimesSeconds.add(ball.getFlightTimeSeconds());
            if (completedFlightTimesSeconds.size() > MAX_COMPLETED_FLIGHT_TIMES) {
              completedFlightTimesSeconds.remove(0);
            }
          }
          return shouldRemove;
        });
  }

  public Translation3d[] getPositions() {
    Translation3d[] t = new Translation3d[fuel.size()];
    for (int i = 0; i < fuel.size(); i++) {
      t[i] = fuel.get(i).position;
    }
    return t;
  }

  public double[] getActiveFlightTimesSeconds() {
    double[] times = new double[fuel.size()];
    for (int i = 0; i < fuel.size(); i++) {
      times[i] = fuel.get(i).getFlightTimeSeconds();
    }
    return times;
  }

  public double[] getCompletedFlightTimesSeconds() {
    int count = Math.min(MAX_COMPLETED_FLIGHT_TIMES, completedFlightTimesSeconds.size());
    double[] times = new double[count];
    int start = completedFlightTimesSeconds.size() - count;
    for (int i = 0; i < count; i++) {
      times[i] = completedFlightTimesSeconds.get(start + i);
    }
    return times;
  }

  public double[] getCompletedTargetHits() {
    int count = Math.min(MAX_COMPLETED_FLIGHT_TIMES, completedTargetHits.size());
    double[] hits = new double[count];
    int start = completedTargetHits.size() - count;
    for (int i = 0; i < count; i++) {
      hits[i] = completedTargetHits.get(start + i);
    }
    return hits;
  }
}
