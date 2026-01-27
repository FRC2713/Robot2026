package frc2713.robot.util;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;

public class BallTrajectorySim {
  public static class Ball {
    // Position is Translation3d (Best for Visualization/Field2d)
    public Translation3d position;

    // Velocity is Translation3d (can be treated like a vector)
    public Translation3d velocity;

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
      this.velocity = new Translation3d();
    }

    public void launch(
        Translation3d startPosition, // Input is now Translation3d
        Translation3d velocity,
        AngularVelocity spinSpeed) {

      this.position = startPosition;
      this.spinRateY = spinSpeed.in(RadiansPerSecond);

      this.velocity = velocity;
    }

    public void update(Time timeStep) {
      double dt = timeStep.in(Seconds);
      double airDensity = 1.225;

      // 1. Gravity
      Translation3d gravityForce = new Translation3d(0, 0, -9.81 * massKg);

      // 2. Drag
      double vMag = velocity.getNorm();
      Translation3d dragForce =
          (vMag < 1e-6)
              ? new Translation3d()
              : velocity.times(-0.5 * airDensity * areaM2 * dragCoeff * vMag);

      // 3. Magnus Lift (CORRECTED)
      Translation3d magnusForce = new Translation3d();

      // We assume "spinRateY" actually means "Backspin Rate" regardless of direction.
      // To get the Backspin Axis, we cross Global Z (Up) with Velocity.
      // This creates a vector pointing to the "Left" relative to travel.
      Translation3d up = new Translation3d(0, 0, 1);

      // Use the .cross() method (returns Vector<N3>)
      var axisVec = up.cross(velocity);

      // Convert Vector<N3> back to Translation3d
      Translation3d spinAxis =
          new Translation3d(axisVec.get(0, 0), axisVec.get(1, 0), axisVec.get(2, 0));

      // Only apply if we have horizontal movement (prevent divide by zero)
      if (spinAxis.getNorm() > 1e-6) {
        // Normalize the axis to length 1, then scale by spin rate
        Translation3d spinVec = spinAxis.div(spinAxis.getNorm()).times(spinRateY);

        // Calculate Magnus Direction: Spin x Velocity
        var magnusVec = spinVec.cross(velocity);
        Translation3d magnusDir =
            new Translation3d(magnusVec.get(0, 0), magnusVec.get(1, 0), magnusVec.get(2, 0));

        if (magnusDir.getNorm() > 1e-6) {
          magnusDir = magnusDir.div(magnusDir.getNorm());
          double liftMagnitude = 0.5 * airDensity * areaM2 * liftCoeff * (vMag * vMag);
          magnusForce = magnusDir.times(liftMagnitude);
        }
      }

      // 4. Integration
      // Make sure to UNCOMMENT the magnusForce!
      Translation3d totalForce = gravityForce.plus(dragForce); // .plus(magnusForce);
      Translation3d acceleration = totalForce.div(massKg);

      velocity = velocity.plus(acceleration.times(dt));
      position = position.plus(velocity.times(dt));
    }

    public Translation3d getPosition() {
      return position;
    }

    public Translation3d getVelocity() {
      return velocity;
    }
  }
}
