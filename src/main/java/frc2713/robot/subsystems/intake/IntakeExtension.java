package frc2713.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

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
import org.littletonrobotics.junction.Logger;

public class IntakeExtension extends MotorSubsystem<MotorInputsAutoLogged, MotorIO>
    implements ArticulatedComponent {

  public IntakeExtension(
      final TalonFXSubsystemConfig config, final MotorIO intakeExtensionMotorIO) {
    super(config, new MotorInputsAutoLogged(), intakeExtensionMotorIO);
  }

  /**
   * Move to specified distance with motion magic
   *
   * @param desiredDistance
   * @return
   */
  public Command setDistanceCommand(Supplier<Distance> desiredDistance) {
    return motionMagicSetpointCommand(
        () -> convertSubsystemPositionToMotorPosition(desiredDistance.get()));
  }

  /**
   * Move to the extended position with motion magic
   *
   * @return
   */
  public Command extendCommand() {
    return setDistanceCommand(IntakeConstants.Extension.extendedPosition);
  }

  /**
   * Move to the retracted postion with motion magic
   *
   * @return
   */
  public Command retractCommand() {
    return setDistanceCommand(() -> IntakeConstants.Extension.retractedPosition);
  }

  @Override
  public void periodic() {
    super.periodic();

    double currentDistanceMeters = inputs.position.in(Rotations) / config.unitRotationsPerMeter;
    Logger.recordOutput(pb.makePath("CurrentDistanceMeters"), currentDistanceMeters);

    double setpointDistanceMeters = positionSetpoint.in(Rotations) / config.unitRotationsPerMeter;
    Logger.recordOutput(pb.makePath("SetpointDistanceMeters"), setpointDistanceMeters);
  }

  @Override
  public Transform3d getTransform3d() {
    // inputs.position is in rotations, convert to meters
    // distance = rotations / (rotations per meter)
    Distance distance = Meters.of(inputs.position.in(Rotations) / config.unitRotationsPerMeter);
    Angle sliderAngle = Degrees.of(-4.479515);

    Distance distanceX = distance.times(Math.cos(sliderAngle.in(Radians)));
    Distance distanceZ = distance.times(Math.sin(sliderAngle.in(Radians)));

    return new Transform3d(
        new Translation3d(distanceX.in(Meters), 0, distanceZ.in(Meters)), new Rotation3d());
  }
}
