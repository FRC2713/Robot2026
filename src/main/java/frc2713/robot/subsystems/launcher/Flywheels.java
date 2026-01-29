package frc2713.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import frc2713.lib.io.ArticulatedComponent;
import frc2713.lib.io.MotorIO;
import frc2713.lib.io.MotorInputsAutoLogged;
import frc2713.lib.subsystem.MotorSubsystem;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;
import frc2713.lib.util.RobotTime;
import frc2713.robot.FieldConstants;
import frc2713.robot.subsystems.launcher.LaunchingSolutionManager.LaunchSolution;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Flywheels extends MotorSubsystem<MotorInputsAutoLogged, MotorIO>
    implements ArticulatedComponent {

  private FuelTrajectories fuelTrajectories = new FuelTrajectories();
  private Time lastUpdateTime = RobotTime.getTimestamp();

  public Flywheels(final TalonFXSubsystemConfig config, final MotorIO launcherMotorIO) {
    super(config, new MotorInputsAutoLogged(), launcherMotorIO);
    this.fuelTrajectories = new FuelTrajectories();
  }

  public Command setVelocity(Supplier<AngularVelocity> desiredVelocity) {
    return velocitySetpointCommand(() -> desiredVelocity.get().times(config.unitToRotorRatio));
  }

  public Command stop() {
    return setVelocity(() -> RotationsPerSecond.of(0));
  }

  @Override
  public void periodic() {
    super.periodic();

    var solution = LaunchingSolutionManager.getInstance().getSolution();

    if (solution.isValid()) {
      launchFuel(solution);
    }
    Pose3d globalPose = this.getGlobalPose();
    Logger.recordOutput(
        super.pb.makePath("ball_vector"),
        new Pose3d[] {
          globalPose, globalPose.plus(new Transform3d(new Translation3d(1, 0, 0), new Rotation3d()))
        });
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
    AngularVelocity wheelSpeed = super.getCurrentVelocity().div(config.unitToRotorRatio);
    Distance wheelDiameter = Inches.of(4);
    Distance wheelCircumference = wheelDiameter.times(Math.PI);
    return InchesPerSecond.of(wheelSpeed.in(RotationsPerSecond) * wheelCircumference.in(Inches));
  }

  public LinearVelocity getLaunchVelocity() {
    Distance toGoal = this.getDistance2d(FieldConstants.Hub.innerCenterPoint);
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
    this.fuelTrajectories.launch(
        this.getGlobalPose().getTranslation(), getLaunchVector(solution), RotationsPerSecond.of(0));
  }
}
