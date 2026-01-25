package frc2713.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc2713.lib.io.ArticulatedComponent;
import frc2713.lib.io.MotorInputsAutoLogged;
import frc2713.lib.io.TalonFXIO;
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

  public Command setAngle(Supplier<Angle> desiredAngle) {
    return motionMagicSetpointCommand(
        () -> convertSubsystemPositionToMotorPosition(desiredAngle.get()));
  }

  @Override
  public void periodic() {
    super.periodic();
    Pose3d goalPose = new Pose3d(FieldConstants.Hub.topCenterPoint, new Rotation3d());

    Logger.recordOutput(pb.makePath("goalVector"), new Pose3d[] {this.getGlobalPose(), goalPose});
  }
  ;

  @Override
  public int getModelIndex() {
    return LauncherConstants.Turret.MODEL_INDEX;
  }

  @Override
  public int getParentModelIndex() {
    return LauncherConstants.Turret.PARENT_INDEX;
  }

  @Override
  public Transform3d getTransform3d() {
    Angle rotations = super.getCurrentPosition().times(config.unitToRotorRatio);
    return config.initialTransform.plus(
        new Transform3d(new Translation3d(), new Rotation3d(0, 0, rotations.in(Radians))));
  }
}
