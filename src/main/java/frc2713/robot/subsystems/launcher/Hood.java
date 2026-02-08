package frc2713.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

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
        Angle aimAngle = solution.hoodPitch().getMeasure();
        boolean launchSolutionValid = solution.isValid();
        if (!launchSolutionValid) {
          // Fallback to distance-based lookup
          aimAngle = Degrees.of(LauncherConstants.Hood.angleMap.get(toGoal.in(Meters)));
        }
        Logger.recordOutput(super.pb.makePath("OTF", "solutionIsValid"), launchSolutionValid);
        Logger.recordOutput(super.pb.makePath("OTF", "distanceToGoal"), toGoal);
        Logger.recordOutput(super.pb.makePath("OTF", "aimAngle"), aimAngle);
        return aimAngle;
      };

  @AutoLogOutput
  public boolean atTarget() {
    return this.io.isMagicMotionAtTarget();
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  @Override
  public Transform3d getTransform3d() {
    Angle rotations = super.getCurrentPosition().div(config.unitToRotorRatio);
    Transform3d localTransform =
        new Transform3d(new Translation3d(), new Rotation3d(0, rotations.in(Radians), 0));

    return config.initialTransform.plus(localTransform);
  }

  @Override
  public Translation3d getRelativeAngularVelocity() {
    return new Translation3d(0, super.getCurrentVelocity().in(RadiansPerSecond), 0);
  }
}
