package frc2713.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import frc2713.lib.util.RobotTime;
import frc2713.robot.util.BallTrajectorySim.Ball;
import org.littletonrobotics.junction.Logger;

public class SimulateBall extends Command {
  Mass ballMass = Pounds.of(0.5);
  Distance ballRadius = Inches.of(5.91 / 2); // 37 mm
  Ball ball = new Ball(ballMass, ballRadius, 0.47, 0.2); // mass(kg), radius
  Translation3d startPos = new Translation3d(0, 0, 1); // 1 meter above ground

  LinearVelocity speed = MetersPerSecond.of(10.0);
  AngularVelocity spin = RotationsPerSecond.of(-10.0);
  Angle pitch = Degrees.of(45.0); // 45 degrees = Upward
  Angle yaw = Degrees.of(0.0); // 0 degrees = Straight down X axis
  Time lastTimeStamp = RobotTime.getTimestamp();

  public SimulateBall() {
    super();
  }
  ;

  @Override
  public void initialize() {
    System.out.println("Starting Ball Trajectory Simulation...");
    lastTimeStamp = RobotTime.getTimestamp();
    ball.launch(startPos, speed, pitch, yaw, spin);
    // Here you would call the BallTrajectorySim main method or equivalent logic
    // BallTrajectorySim.simulate();
  }

  @Override
  public void execute() {
    Time currentTime = RobotTime.getTimestamp();
    Time deltaTime = currentTime.minus(lastTimeStamp);
    lastTimeStamp = currentTime;
    ball.update(deltaTime);
    Logger.recordOutput("SimulatedBall/Pose", ball.position);
    // Simulation logic would go here if needed
  }

  @Override
  public boolean isFinished() {
    return ball.position.getZ() <= 0;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Ball Trajectory Simulation Ended.");
  }
}
