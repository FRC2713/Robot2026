package frc2713.robot.subsystems.serializer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc2713.lib.io.MotorIO;
import frc2713.lib.io.MotorInputsAutoLogged;
import frc2713.lib.subsystem.MotorSubsystem;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class Feeder extends MotorSubsystem<MotorInputsAutoLogged, MotorIO> {

  public Feeder(final TalonFXSubsystemConfig config, final MotorIO feederMotorIO) {
    super(config, new MotorInputsAutoLogged(), feederMotorIO);
  }

  public Command setVelocity(Supplier<AngularVelocity> desiredVelocity) {
    return velocitySetpointCommand(desiredVelocity);
  }

  public Command feedShooter() {
    return setVelocity(() -> SerializerConstants.Feeder.shootingSpeed.get());
  }

  public Command feedWhenReady(BooleanSupplier isReady) {
    return Commands.sequence(
        Commands.waitUntil(isReady), setVelocity(SerializerConstants.Feeder.shootingSpeed));
    // return setVelocity(
    //     () ->
    //         isReady.getAsBoolean()
    //             ? SerializerConstants.Feeder.shootingSpeed.get()
    //             : RotationsPerSecond.of(0));
  }

  public Command stop() {
    return setVelocity(() -> RotationsPerSecond.of(0));
  }

  @Override
  public void periodic() {
    super.periodic();
    // Additional periodic code for feeder can be added here
  }
}
