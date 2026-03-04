package frc2713.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc2713.lib.io.ArticulatedComponent;
import frc2713.lib.io.MotorIO;
import frc2713.lib.io.MotorInputsAutoLogged;
import frc2713.lib.subsystem.MotorFollowerSubsystem;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;
import frc2713.lib.util.RobotTime;
import frc2713.robot.subsystems.launcher.LaunchingSolutionManager.LaunchSolution;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheels extends MotorFollowerSubsystem<MotorInputsAutoLogged, MotorIO>
    implements ArticulatedComponent {

  private FuelTrajectories fuelTrajectories = new FuelTrajectories();
  private Time lastUpdateTime = RobotTime.getTimestamp();
  private Time lastLaunchTime = RobotTime.getTimestamp();

  public Flywheels(
      final TalonFXSubsystemConfig leaderConfig,
      final TalonFXSubsystemConfig followerConfig,
      final MotorIO leaderLauncherMotorIO,
      final MotorIO followerLauncherMotorIO) {
    super(
        "Flywheels",
        leaderConfig,
        followerConfig,
        new MotorInputsAutoLogged(),
        new MotorInputsAutoLogged(),
        leaderLauncherMotorIO,
        followerLauncherMotorIO);
    this.fuelTrajectories = new FuelTrajectories();
  }

  public Command setVelocity(Supplier<AngularVelocity> desiredVelocity) {
    return velocitySetpointCommand(desiredVelocity);
  }

  public Command setVelocityUntilTarget(Supplier<AngularVelocity> desiredVelocity) {
    return velocitySetpointUntilOnTargetCommand(
        desiredVelocity, () -> LauncherConstants.Flywheels.acceptableError);
  }

  public Command stop() {
    return setVelocity(() -> RotationsPerSecond.of(0));
  }

  public Command hubCommand() {
    return setVelocity(() -> LauncherConstants.Flywheels.staticHubVelocity);
  }

  /** Command to continuously track the on-the-fly flywheel velocity */
  public Command idleSpeedCommand() {
    return setVelocity(() -> LauncherConstants.Flywheels.idleVelocity);
  }

  /**
   * Supplier that continuously calculates the on-the-fly flywheel velocity. Uses the launch
   * solution if valid, otherwise falls back to distance-based lookup.
   */
  public final Supplier<AngularVelocity> otfVelocitySupplier =
      () -> {
        var solution = LaunchingSolutionManager.getInstance().getSolution();
        Distance toGoal = this.getDistance2d(LaunchingSolutionManager.currentGoal);
        boolean solutionIsValid = solution.isValid();

        LinearVelocity targetSurfaceSpeed;
        if (solutionIsValid) {
          targetSurfaceSpeed = MetersPerSecond.of(solution.flywheelSpeedMetersPerSecond());
          Logger.recordOutput(super.pb.makePath("OTF", "response"), "using solution");
        } else if (solution.effectiveDistanceMeters() <= 0.9) {
          // invalid bc we're too close
          Logger.recordOutput(super.pb.makePath("OTF", "response"), "hub shot");
          targetSurfaceSpeed = FeetPerSecond.of(5);

        } else {
          // Fallback to distance-based lookup
          Logger.recordOutput(super.pb.makePath("OTF", "response"), "lookup map");
          targetSurfaceSpeed =
              FeetPerSecond.of(LauncherConstants.Flywheels.velocityMap.get(toGoal.in(Meters)));
        }

        // Convert surface speed to angular velocity: omega = v / r
        double wheelRadiusMeters = LauncherConstants.Flywheels.WHEEL_DIAMETER.div(2).in(Meters);
        double surfaceSpeedMps = targetSurfaceSpeed.in(MetersPerSecond);
        AngularVelocity targetVelocity =
            RotationsPerSecond.of(surfaceSpeedMps / (wheelRadiusMeters * 2 * Math.PI));

        Logger.recordOutput(super.pb.makePath("OTF", "solutionIsValid"), solutionIsValid);
        Logger.recordOutput(super.pb.makePath("OTF", "distanceToGoal"), toGoal);
        Logger.recordOutput(super.pb.makePath("OTF", "targetSurfaceSpeed"), targetSurfaceSpeed);
        Logger.recordOutput(super.pb.makePath("OTF", "targetVelocity"), targetVelocity);
        return targetVelocity;
      };

  public Command otfCommand() {
    return setVelocity(otfVelocitySupplier);
  }

  public Command simulateLaunchedFuel(BooleanSupplier isReady) {
    return Commands.run(
        () -> {
          if (isReady.getAsBoolean())
            this.launchFuel(LaunchingSolutionManager.getInstance().getSolution());
        });
  }

  @AutoLogOutput
  public boolean atTarget() {
    return Math.abs(inputs.closedLoopError)
        <= LauncherConstants.Flywheels.acceptableError.in(RotationsPerSecond);
  }

  @Override
  public void periodic() {
    super.periodic();

    // update ball_vector visualization
    Pose3d globalPose = this.getGlobalPose();
    Logger.recordOutput(
        super.pb.makePath("ball_vector"),
        new Pose3d[] {
          globalPose, globalPose.plus(new Transform3d(new Translation3d(1, 0, 0), new Rotation3d()))
        });

    // update fuel trajectories physics for balls already in flight
    Time now = RobotTime.getTimestamp();
    Time dt = now.minus(lastUpdateTime);
    fuelTrajectories.update(dt);
    this.lastUpdateTime = now;
    Logger.recordOutput(pb.makePath("fuel_trajectories"), fuelTrajectories.getPositions());
  }

  @Override
  public Transform3d getTransform3d() {
    return LauncherConstants.Flywheels.localTransform;
  }

  @Override
  public Translation3d getRelativeLinearVelocity() {
    return new Translation3d(0, 0, 0);
  }

  public LinearVelocity getSurfaceSpeed() {
    AngularVelocity wheelSpeed = getLeaderCurrentVelocity();
    Distance wheelDiameter = Inches.of(4);
    Distance wheelCircumference = wheelDiameter.times(Math.PI);
    return InchesPerSecond.of(wheelSpeed.in(RotationsPerSecond) * wheelCircumference.in(Inches));
  }

  public LinearVelocity getLaunchVelocity() {
    Distance toGoal = this.getDistance2d(LaunchingSolutionManager.currentGoal);
    LinearVelocity vel =
        FeetPerSecond.of(LauncherConstants.Flywheels.velocityMap.get(toGoal.in(Meters)));
    Logger.recordOutput(pb.makePath("launchVelocity"), vel);
    return vel;
  }

  public LinearVelocity getOnTheFlyLaunchVelocity(LaunchSolution solution) {

    if (solution.isValid()) {
      double targetSpeedMps = solution.flywheelSpeedMetersPerSecond();
      Logger.recordOutput(super.pb.makePath("launchVelocity"), targetSpeedMps);
      return MetersPerSecond.of(targetSpeedMps);
    }
    return MetersPerSecond.of(0);
  }

  public Translation3d getLaunchVector(LaunchSolution solution) {
    // A. Robot Structure Velocity (Drive + Turret Spin + Hood Pitch)
    // If the robot is driving 2m/s North, this returns (0, 2, 0)
    Translation3d structureVel = this.getGlobalLinearVelocity();

    // B. Muzzle Velocity (Shot Power)

    // Define the launch speed relative to the flywheel (Forward X)
    Translation3d flywheelVelRel =
        new Translation3d(getOnTheFlyLaunchVelocity(solution).in(MetersPerSecond), 0, 0);

    // C. Rotate Muzzle Velocity to Global Frame
    // We use the Global Rotation of the flywheels (which includes Drive, Turret, Hood)
    Rotation3d launcherFacing = this.getGlobalPose().getRotation();
    Translation3d launchVelGlobal = flywheelVelRel.rotateBy(launcherFacing);

    // D. Combine: V_ball = V_robot + V_shot
    return structureVel.plus(launchVelGlobal);
  }

  public void launchFuel(LaunchSolution solution) {
    Time now = RobotTime.getTimestamp();
    // Enforce max fire rate by checking time since last launch. If we haven't waited long enough,
    // skip this launch.
    if (now.minus(lastLaunchTime).in(Seconds)
        >= (1.0 / LauncherConstants.Flywheels.launchRateFuelPerSecond)) {
      lastLaunchTime = now;

      this.fuelTrajectories.launch(
          this.getGlobalPose().getTranslation(),
          getLaunchVector(solution),
          RotationsPerSecond.of(0));
    }
  }
}
