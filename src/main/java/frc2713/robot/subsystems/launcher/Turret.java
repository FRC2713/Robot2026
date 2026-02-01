package frc2713.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc2713.lib.io.ArticulatedComponent;
import frc2713.lib.io.MotorIO;
import frc2713.lib.io.MotorInputsAutoLogged;
import frc2713.lib.subsystem.KinematicsManager;
import frc2713.lib.subsystem.MotorSubsystem;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;
import frc2713.robot.FieldConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends MotorSubsystem<MotorInputsAutoLogged, MotorIO>
    implements ArticulatedComponent {

  public Turret(final TalonFXSubsystemConfig config, final MotorIO turretMotorIO) {
    super(config, new MotorInputsAutoLogged(), turretMotorIO);
  }

  public Command setAngle(Supplier<Angle> desiredAngle) {
    return motionMagicSetpointCommand(
        () -> convertSubsystemPositionToMotorPosition(desiredAngle.get()));
  }

  /**
   * Supplier that continuously calculates the on-the-fly turret angle. Uses the launch solution if
   * valid, otherwise falls back to simple hub tracking.
   */
  public final Supplier<Angle> otfAngleSupplier =
      () -> {
        Angle angle = getLauncOnTheFlyAngle();
        // If no valid solution, fall back to simple hub angle
        boolean solutionIsValid = true;
        if (angle.equals(Degrees.of(0))) {
          solutionIsValid = false;
          angle = getHubAngle();
        }
        Logger.recordOutput(super.pb.makePath("OTF", "solutionIsValid"), solutionIsValid);
        Logger.recordOutput(pb.makePath("OTF", "targetAngle"), angle);
        return angle;
      };

  public Command oftCommand() {
    return setAngle(otfAngleSupplier);
  }

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
    Angle rotations = super.getCurrentPosition().times(config.unitToRotorRatio);
    return config.initialTransform.plus(
        new Transform3d(new Translation3d(), new Rotation3d(0, 0, rotations.in(Radians))));
  }

  @Override
  public Translation3d getRelativeAngularVelocity() {
    return new Translation3d(0, 0, super.getCurrentVelocity().in(RadiansPerSecond));
  }

  public Angle getHubAngle() {
    Translation3d diff = this.getTranslationTo(LaunchingSolutionManager.currentGoal.positionalTarget());
    // 2. Calculate the Global Yaw needed to face the target
    // Math.atan2(y, x) handles all quadrants correctly
    double globalTargetRadians = Math.atan2(diff.getY(), diff.getX());

    // 3. Get the Chassis Heading (Global)
    // You need the Robot's orientation on the field to make this relative.
    // Assuming ID 0 is your Drive/Chassis in KinematicsManager:
    Rotation3d chassisRotation = KinematicsManager.getInstance().getGlobalPose(0).getRotation();
    double chassisHeadingRadians = chassisRotation.getZ();

    // 4. Calculate Relative Angle (Target - Chassis)
    double relativeRadians = globalTargetRadians - chassisHeadingRadians;

    // 5. Normalize to range (-PI to PI) so the turret takes the shortest path
    // e.g., if result is 350 degrees, this turns it into -10 degrees
    double normalizedRadians = MathUtil.angleModulus(relativeRadians);

    return Radians.of(normalizedRadians);
  }

  public Angle getLauncOnTheFlyAngle() {
    // 1. Get the latest solution
    var solution = LaunchingSolutionManager.getInstance().getSolution();

    if (solution.isValid()) {
      // 2. Convert Field-Relative Goal to Robot-Relative Setpoint

      // Get Chassis Heading from Kinematics
      Rotation2d chassisHeading =
          KinematicsManager.getInstance().getGlobalPose(0).getRotation().toRotation2d();

      // TargetYaw - ChassisYaw = TurretSetpoint
      Rotation2d localSetpoint = solution.turretFieldRelativeYaw().minus(chassisHeading);

      return localSetpoint.getMeasure();
    }
    return Degrees.of(0);
  }
}
