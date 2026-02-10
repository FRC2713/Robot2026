package frc2713.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc2713.robot.subsystems.launcher.LauncherConstants.Hood.FORWARD_LIMIT_DEGREES;
import static frc2713.robot.subsystems.launcher.LauncherConstants.Hood.REVERSE_LIMIT_DEGREES;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc2713.lib.io.ArticulatedComponent;
import frc2713.lib.io.MotorIO;
import frc2713.lib.io.MotorInputsAutoLogged;
import frc2713.lib.subsystem.MotorSubsystem;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood extends MotorSubsystem<MotorInputsAutoLogged, MotorIO>
    implements ArticulatedComponent {

  public Hood(final TalonFXSubsystemConfig config, final MotorIO launcherMotorIO) {
    super(config, new MotorInputsAutoLogged(), launcherMotorIO);
  }

  public Command setAngleCommand(Supplier<Angle> desiredAngle) {
    return motionMagicSetpointCommand(
        () -> convertSubsystemPositionToMotorPosition(desiredAngle.get()));
  }

  public Command retract() {
    return setAngleCommand(() -> LauncherConstants.Hood.retractedPosition);
  }

  public Command hubCommand() {
    return setAngleCommand(() -> LauncherConstants.Hood.staticHubAngle);
  }

  public Command otfCommand() {
    return setAngleCommand(otfAngSupplier);
  }

  public final Supplier<Angle> otfAngSupplier =
      () -> {
        var solution = LaunchingSolutionManager.getInstance().getSolution();
        Distance toGoal =
            this.getDistance2d(LaunchingSolutionManager.currentGoal.positionalTarget());
        boolean launchSolutionValid = solution.isValid();

        Angle aimAngle;
        if (launchSolutionValid) {
          aimAngle = solution.hoodPitch().getMeasure();
        } else {
          // Fallback to distance-based lookup
          aimAngle = Degrees.of(LauncherConstants.Hood.angleMap.get(toGoal.in(Meters)));
        }

        Logger.recordOutput(super.pb.makePath("OTF", "solutionIsValid"), launchSolutionValid);
        Logger.recordOutput(super.pb.makePath("OTF", "distanceToGoal"), toGoal);
        Logger.recordOutput(super.pb.makePath("OTF", "aimAngleDeg"), aimAngle.in(Degrees));
        Logger.recordOutput(
            super.pb.makePath("OTF", "currentAngleDeg"), inputs.position.in(Degrees));
        Logger.recordOutput(
            super.pb.makePath("OTF", "lookupAngle"),
            Degrees.of(LauncherConstants.Hood.angleMap.get(toGoal.in(Meters))));
        return aimAngle;
      };

  /**
   * Like {@link #setAngleStopAtBounds}, but allows scaling the velocity and acceleration based on
   * an input (e.g., trigger pressure).
   *
   * @param desiredAngle The desired angle supplier
   * @param velocityScale Scale factor for velocity and acceleration (0.0 to 1.0)
   */
  public Command setAngleStopAtBounds(
      Supplier<Angle> desiredAngle, Supplier<Double> velocityScale) {
    return motionMagicSetpointCommand(
        () -> {
          double commandedDegrees = desiredAngle.get().in(Degrees);

          // Clamp directly to turret limits instead of wrapping
          double clampedDegrees =
              MathUtil.clamp(commandedDegrees, REVERSE_LIMIT_DEGREES, FORWARD_LIMIT_DEGREES);

          Logger.recordOutput(pb.makePath("setpoint", "commandedDegrees"), commandedDegrees);
          Logger.recordOutput(pb.makePath("setpoint", "clampedDegrees"), clampedDegrees);

          return convertSubsystemPositionToMotorPosition(Degrees.of(clampedDegrees));
        },
        () -> {
          var mmConfig = new com.ctre.phoenix6.configs.MotionMagicConfigs();
          double scale = MathUtil.clamp(velocityScale.get(), 0.0, 1.0);

          // Scale velocity and acceleration based on input
          mmConfig.MotionMagicCruiseVelocity =
              config.fxConfig.MotionMagic.MotionMagicCruiseVelocity * scale;
          mmConfig.MotionMagicAcceleration =
              config.fxConfig.MotionMagic.MotionMagicAcceleration * scale;
          mmConfig.MotionMagicJerk = config.fxConfig.MotionMagic.MotionMagicJerk;

          Logger.recordOutput(pb.makePath("setpoint", "velocityScale"), scale);

          return mmConfig;
        },
        0);
  }

  @AutoLogOutput
  public boolean atTarget() {
    return this.inputs.isMotionMagicAtTarget;
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  @Override
  public Transform3d getTransform3d() {
    Angle rotations = super.getCurrentPosition();
    Transform3d localTransform =
        new Transform3d(new Translation3d(), new Rotation3d(0, rotations.in(Radians), 0));

    return config.initialTransform.plus(localTransform);
  }

  @Override
  public Translation3d getRelativeAngularVelocity() {
    return new Translation3d(0, super.getCurrentVelocity().in(RadiansPerSecond), 0);
  }
}
