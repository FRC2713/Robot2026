package frc2713.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import frc2713.lib.io.ArticulatedComponent;
import frc2713.lib.io.MotorIO;
import frc2713.lib.io.MotorInputsAutoLogged;
import frc2713.lib.subsystem.MotorSubsystem;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;

public class Hood extends MotorSubsystem<MotorInputsAutoLogged, MotorIO>
    implements ArticulatedComponent {

  public Hood(final TalonFXSubsystemConfig config, final MotorIO launcherMotorIO) {
    super(config, new MotorInputsAutoLogged(), launcherMotorIO);
  }

  @Override
  public void periodic() {
    super.periodic();
    // Additional periodic code for Hood can be added here
  }

  @Override
  public int getModelIndex() {
    return 3;
  }

  @Override
  public int getParentModelIndex() {
    return 2;
  }

  @Override
  public Transform3d getTransform3d() {
    // TODO: Get this from sensors
    // Angle rotations = Rotations.of((Math.sin(Timer.getFPGATimestamp())+1) * .1);
    Angle rotations = Rotations.of(0);

    Transform3d localTransform =
        new Transform3d(new Translation3d(), new Rotation3d(0, rotations.in(Radians), 0));

    return config.initialTransform.plus(localTransform);
  }
}
