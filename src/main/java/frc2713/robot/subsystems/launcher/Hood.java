package frc2713.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc2713.lib.io.ArticulatedComponent;
import frc2713.lib.io.MotorInputsAutoLogged;
import frc2713.lib.io.TalonFXIO;
import frc2713.lib.subsystem.MotorSubsystem;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;
import java.util.function.Supplier;

public class Hood extends MotorSubsystem<MotorInputsAutoLogged, TalonFXIO>
    implements ArticulatedComponent {

  public Hood(final TalonFXSubsystemConfig config, final TalonFXIO launcherMotorIO) {
    super(config, new MotorInputsAutoLogged(), launcherMotorIO);
  }

  public Command setAngleCommand(Supplier<Angle> desiredAngle) {
    return motionMagicSetpointCommand(
        () -> convertSubsystemPositionToMotorPosition(desiredAngle.get()));
  }

  public Command retract() {
    return setAngleCommand(() -> LauncherConstants.Hood.retractedPosition);
  }

  @Override
  public void periodic() {
    super.periodic();
    // Additional periodic code for Hood can be added here
  }

  @Override
  public int getModelIndex() {
    return LauncherConstants.Hood.MODEL_INDEX;
  }

  @Override
  public int getParentModelIndex() {
    return LauncherConstants.Hood.PARENT_INDEX;
  }

  @Override
  public Transform3d getTransform3d() {
    // TODO: Get this from sensors
    Angle rotations = Rotations.of((Math.sin(Timer.getFPGATimestamp()) + 1) * .1);
    Transform3d localTransform =
        new Transform3d(new Translation3d(), new Rotation3d(0, rotations.in(Radians), 0));

    return config.initialTransform.plus(localTransform);
  }
}
