package frc2713.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
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
import frc2713.robot.FieldConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood extends MotorSubsystem<MotorInputsAutoLogged, MotorIO>
    implements ArticulatedComponent {

  public Hood(final TalonFXSubsystemConfig config, final MotorIO launcherMotorIO) {
    super(config, new MotorInputsAutoLogged(), launcherMotorIO);
  }

  public Command setAngleCommand(Supplier<Angle> desiredAngle) {
    return motionMagicSetpointCommand(desiredAngle);
  }

  public Command retract() {
    return setAngleCommand(() -> LauncherConstants.Hood.retractedPosition);
  }

  public Command hubCommand() {
    return setAngleCommand(LauncherConstants.Hood.staticHubAngle);
  }

  public Command otfCommand() {
    return setAngleCommand(otfAngSupplier);
  }

  /**
   * Creates a command that automatically retracts the hood when in designated field zones and
   * raises it back up when outside those zones.
   *
   * @param poseSupplier Supplier for the robot's current pose
   * @param defaultAngleSupplier Supplier for the desired hood angle when not in a retraction zone
   * @return A command that manages hood position based on field location
   */
  public Command autoRetractCommand(
      Supplier<Pose2d> poseSupplier, Supplier<Angle> defaultAngleSupplier) {
    return setAngleCommand(
        () -> {
          Pose2d currentPose = poseSupplier.get();
          boolean inRetractionZone =
              FieldConstants.HoodRetractionZones.isInRetractionZone(currentPose);

          Logger.recordOutput(pb.makePath("AutoRetract", "inRetractionZone"), inRetractionZone);
          ducking = inRetractionZone;

          if (inRetractionZone) {
            return LauncherConstants.Hood.retractedPosition;
          } else {
            return defaultAngleSupplier.get();
          }
        });
  }

  @AutoLogOutput public boolean ducking = false;

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
