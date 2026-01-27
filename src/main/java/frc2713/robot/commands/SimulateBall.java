package frc2713.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import frc2713.lib.util.RobotTime;
import frc2713.robot.util.BallTrajectorySim.Ball;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SimulateBall extends Command {
  Mass ballMass = Pounds.of(0.5);
  Distance ballRadius = Inches.of(5.91 / 2); // 37 mm
  Ball ball = new Ball(ballMass, ballRadius, 0.47, 0.2); // mass(kg), radius
  Supplier<Translation3d> initialPosSupplier = () -> new Translation3d();
  Supplier<Translation3d> initialVelSupplier = () -> new Translation3d();
  Supplier<AngularVelocity> initialAngularVelSupplier = () -> RotationsPerSecond.of(0.0);

  Time lastTimeStamp = RobotTime.getTimestamp();

  public SimulateBall() {
    super();
  }
  ;

  public SimulateBall(
      Supplier<Translation3d> initialPosSupplier,
      Supplier<Translation3d> initialVelSupplier,
      Supplier<AngularVelocity> initialAngularVelSupplier) {
    this.initialPosSupplier = initialPosSupplier;
    this.initialVelSupplier = initialVelSupplier;
    this.initialAngularVelSupplier = initialAngularVelSupplier;
  }

  @Override
  public void initialize() {
    System.out.println("Starting Ball Trajectory Simulation...");
    lastTimeStamp = RobotTime.getTimestamp();
    ball.launch(
        initialPosSupplier.get(), initialVelSupplier.get(), initialAngularVelSupplier.get());
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
