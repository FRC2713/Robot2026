package frc2713.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc2713.lib.io.ArticulatedComponent;
import frc2713.lib.io.MotorInputsAutoLogged;
import frc2713.lib.io.TalonFXIO;
import frc2713.lib.subsystem.MotorSubsystem;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Flywheels extends MotorSubsystem<MotorInputsAutoLogged, TalonFXIO>
    implements ArticulatedComponent {

  public Flywheels(final TalonFXSubsystemConfig config, final TalonFXIO launcherMotorIO) {
    super(config, new MotorInputsAutoLogged(), launcherMotorIO);
  }

  public Command setVelocity(Supplier<AngularVelocity> desiredVelocity) {
    return velocitySetpointCommand(() -> desiredVelocity.get().times(config.unitToRotorRatio));
  }

  public Command stop() {
    return setVelocity(() -> RotationsPerSecond.of(0));
  }

  @Override
  public void periodic() {
    super.periodic();
    Pose3d globalPose = this.getGlobalPose();
    Logger.recordOutput(
        super.pb.makePath("ball_vector"),
        new Pose3d[] {
          globalPose,
          globalPose.plus(new Transform3d(new Translation3d(1, 0, 0), new Rotation3d()))
        });
  }

  @Override
  public Transform3d getTransform3d() {
    return new Transform3d(
        new Translation3d(Inches.of(-5).in(Meters), 0, Inches.of(2).in(Meters)),
        new Rotation3d(0, Degrees.of(-90).in(Radians), 0));
  }

  @Override
  public Vector<N3> getRelativeLinearVelocity() {
    LinearVelocity surfaceSpeed = FeetPerSecond.of(10);
    return VecBuilder.fill(surfaceSpeed.in(MetersPerSecond), 0, 0);
  }
}
