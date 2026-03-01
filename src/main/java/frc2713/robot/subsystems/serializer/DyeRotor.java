package frc2713.robot.subsystems.serializer;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc2713.lib.io.ArticulatedComponent;
import frc2713.lib.io.MotorIO;
import frc2713.lib.io.MotorInputsAutoLogged;
import frc2713.lib.subsystem.MotorSubsystem;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class DyeRotor extends MotorSubsystem<MotorInputsAutoLogged, MotorIO>
    implements ArticulatedComponent {

  public DyeRotor(final TalonFXSubsystemConfig config, final MotorIO indexerMotorIO) {
    super(config, new MotorInputsAutoLogged(), indexerMotorIO);
  }

  public Command setVelocity(Supplier<AngularVelocity> desiredVelocity) {
    return velocitySetpointCommand(desiredVelocity);
  }

  public Command indexFuel() {
    return setVelocity(() -> SerializerConstants.DyeRotor.indexingSpeed.get());
  }

  public Command feedWhenReady(BooleanSupplier isReady) {
    return Commands.sequence(
        Commands.waitUntil(isReady), setVelocity(SerializerConstants.DyeRotor.indexingSpeed));
    // return setVelocity(
    //     () ->
    //         isReady.getAsBoolean()
    //             ? SerializerConstants.DyeRotor.indexingSpeed.get()
    //             : RotationsPerSecond.of(0));
  }

  public Command outtakeFuel() {
    return setVelocity(() -> SerializerConstants.DyeRotor.outdexingSpeed);
  }

  public Command stopCommand() {
    return setVelocity(() -> RotationsPerSecond.of(0));
  }

  @Override
  public void periodic() {
    super.periodic();
    // Additional periodic code for indexer can be added here
  }

  @Override
  public Transform3d getTransform3d() {
    Angle rotations = super.getCurrentPosition();
    Transform3d localTransform =
        new Transform3d(new Translation3d(), new Rotation3d(0, 0, rotations.in(Radians)));

    return config.initialTransform.plus(localTransform);
  }
}
