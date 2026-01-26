package frc2713.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc2713.lib.io.ArticulatedComponent;
import frc2713.lib.io.MotorInputsAutoLogged;
import frc2713.lib.io.TalonFXIO;
import frc2713.lib.subsystem.MotorSubsystem;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;
import java.util.function.Supplier;

public class IntakeExtension extends MotorSubsystem<MotorInputsAutoLogged, TalonFXIO>
    implements ArticulatedComponent {

  public IntakeExtension(
      final TalonFXSubsystemConfig config, final TalonFXIO intakeExtensionMotorIO) {
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
    return setDistanceCommand(() -> IntakeConstants.Extension.extendedPostion);
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
    // Additional periodic code for intake can be added here
  }

  @Override
  public Transform3d getTransform3d() {
    // TODO: Get this from sensors
    Distance distance = Inches.of(Math.sin(Timer.getFPGATimestamp()) + 1).times(6);
    Angle sliderAngle = Degrees.of(-4.479515);

    Distance distanceX = distance.times(Math.cos(sliderAngle.in(Radians)));
    Distance distanceZ = distance.times(Math.sin(sliderAngle.in(Radians)));

    return new Transform3d(
        new Translation3d(distanceX.in(Meters), 0, distanceZ.in(Meters)), new Rotation3d());
  }
}
