package frc2713.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc2713.robot.subsystems.launcher.LauncherConstants.Turret.FORWARD_LIMIT_DEGREES;
import static frc2713.robot.subsystems.launcher.LauncherConstants.Turret.REVERSE_LIMIT_DEGREES;
import static frc2713.robot.subsystems.launcher.LauncherConstants.Turret.SLOPE;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc2713.lib.io.ArticulatedComponent;
import frc2713.lib.io.MotorInputsAutoLogged;
import frc2713.lib.io.TalonFXIO;
import frc2713.lib.subsystem.KinematicsManager;
import frc2713.lib.subsystem.MotorSubsystem;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;
import frc2713.robot.FieldConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends MotorSubsystem<MotorInputsAutoLogged, TalonFXIO>
    implements ArticulatedComponent {

  public Turret(final TalonFXSubsystemConfig config, final TalonFXIO turretMotorIO) {
    super(config, new MotorInputsAutoLogged(), turretMotorIO);
  }

  public static double turretPositionFromEncoders(double e1, double e2) {
      // 1. Calculate the 'Difference' (The Phase)
      double diff = e2 - e1;
      
      // Normalize to [-180, 180]. This is the "Vernier Lock"
      while (diff <= -180) diff += 360;
      while (diff > 180) diff -= 360;

      // 2. Coarse Estimate (The "Big Picture")
      // This uses the phase shift to guess the rough position.
      double coarseAngle = diff * SLOPE;

      // 3. The Ratio (How many times E1 spins per 1 Turret degree)
      double encoderToTurretRatio = 8.5;
      
      // 4. Lap Calculation (The "Fine" logic)
      // We calculate how many full 360s E1 has likely traveled.
      // 'expectedE1' is where the encoder SHOULD be if coarseAngle was perfect.
      double expectedE1 = coarseAngle * encoderToTurretRatio;
      
      // Determine the 'Lap' by finding how many 360s are between 
      // the expected position and the actual sensor reading (e1).
      double lap = Math.round((expectedE1 - e1) / 360.0);
      
      // 5. High-Resolution Output
      // Combine the Lap (coarse) with the Sensor Reading (fine)
      return (lap * 360.0 + e1) / encoderToTurretRatio;
  }
  public static double convertToClosestBoundedTurretAngleDegrees(
            double targetAngleDegrees, Rotation2d current) {
        double currentTotalRadians = (current.getRotations() * 2 * Math.PI);
        double closestOffset = Units.degreesToRadians(targetAngleDegrees) - current.getRadians();
        if (closestOffset > Math.PI) {

            closestOffset -= 2 * Math.PI;

        } else if (closestOffset < -Math.PI) {
            closestOffset += 2 * Math.PI;
        }

        double finalOffset = currentTotalRadians + closestOffset;
        if ((currentTotalRadians + closestOffset) % (2 * Math.PI)
                == (currentTotalRadians - closestOffset)
                        % (2 * Math.PI)) { // If the offset can go either way, go closer to zero
            if (finalOffset > 0) {
                finalOffset = currentTotalRadians - Math.abs(closestOffset);
            } else {
                finalOffset = currentTotalRadians + Math.abs(closestOffset);
            }
        }
        if (finalOffset > Units.degreesToRadians(FORWARD_LIMIT_DEGREES)) { // if past upper rotation limit
            finalOffset -= (2 * Math.PI);
        } else if (finalOffset < Units.degreesToRadians(REVERSE_LIMIT_DEGREES)) { // if below lower rotation limit
            finalOffset += (2 * Math.PI);
        }

        return Units.radiansToDegrees(finalOffset);
  }
  public Command setAngle(Supplier<Angle> desiredAngle) {
    return motionMagicSetpointCommand(
        () -> convertSubsystemPositionToMotorPosition(desiredAngle.get()));
  }

  @Override
  public void periodic() {
    super.periodic();
    Pose3d goalPose = new Pose3d(FieldConstants.Hub.topCenterPoint, new Rotation3d());

    Logger.recordOutput(pb.makePath("goalVector"), new Pose3d[] {this.getGlobalPose(), goalPose});
    super.setCurrentPosition(getLauncOnTheFlyAngle());
  }
  ;

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
    Translation3d diff = this.getTranslationTo(FieldConstants.Hub.topCenterPoint);
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
