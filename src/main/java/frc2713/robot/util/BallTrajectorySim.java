package frc2713.robot.util;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;

public class BallTrajectorySim {
  public static class Ball {
    // Position is Translation3d (Best for Visualization/Field2d)
    public Translation3d position;

    // Velocity is Vector<N3> (Best for Physics Math)
    public Vector<N3> velocity;

    // Simplified: Spin is just a scalar speed around the Y-axis.
    // Negative = Backspin (Lift), Positive = Topspin (Dive)
    private double spinRateY;

    double massKg;
    double radiusM;
    double areaM2;
    double dragCoeff;
    double liftCoeff;

    public Ball(Mass mass, Distance radius, double dragCoeff, double liftCoeff) {
      this.massKg = mass.in(Kilograms);
      this.radiusM = radius.in(Meters);
      this.dragCoeff = dragCoeff;
      this.liftCoeff = liftCoeff;
      this.areaM2 = Math.PI * radiusM * radiusM;

      this.position = new Translation3d();
      this.velocity = VecBuilder.fill(0, 0, 0);
    }

    public void launch(
        Translation3d startPosition, // Input is now Translation3d
        Vector<N3> velocity,
        AngularVelocity spinSpeed) {

      this.position = startPosition;
      this.spinRateY = spinSpeed.in(RadiansPerSecond);

      this.velocity = velocity;
    }

    public void update(Time timeStep) {
      double dt = timeStep.in(Seconds);
      double airDensity = 1.225;

      // 1. Gravity
      Vector<N3> gravityForce = VecBuilder.fill(0, 0, -9.81 * massKg);

      // 2. Drag (Standard Formula)
      double vMag = velocity.norm();
      Vector<N3> dragForce = velocity.times(-0.5 * airDensity * areaM2 * dragCoeff * vMag);

      // 3. Magnus Lift (CORRECTED FORMULA)
      // We first calculate the direction of the lift (perpendicular to spin & velocity)
      // Direction = Normalized(Spin x Velocity)
      // Magnitude = 0.5 * rho * Area * CL * v^2

      Vector<N3> spinVec = VecBuilder.fill(0, spinRateY, 0); // Assuming Y-axis spin
      Vector<N3> magnusDir = Vector.cross(spinVec, velocity);

      Vector<N3> magnusForce = VecBuilder.fill(0, 0, 0);

      // Only apply lift if we are moving and spinning
      if (magnusDir.norm() > 1e-6) {
        // Normalize direction so it has length 1.0
        magnusDir = magnusDir.div(magnusDir.norm());

        // Calculate standard Aerodynamic Lift Force
        double liftMagnitude = 0.5 * airDensity * areaM2 * liftCoeff * (vMag * vMag);

        magnusForce = magnusDir.times(liftMagnitude);
      }

      // 4. Integration
      Vector<N3> totalForce = gravityForce.plus(dragForce).plus(magnusForce);
      Vector<N3> acceleration = totalForce.div(massKg);

      velocity = velocity.plus(acceleration.times(dt));

      // Update Position (Bridge to Translation3d)
      Translation3d stepMove =
          new Translation3d(
              velocity.get(0, 0) * dt, velocity.get(1, 0) * dt, velocity.get(2, 0) * dt);
      position = position.plus(stepMove);
    }

    public Translation3d getPosition() {
      return position;
    }
  }
}
