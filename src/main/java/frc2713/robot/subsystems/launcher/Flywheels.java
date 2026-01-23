package frc2713.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc2713.lib.io.MotorInputsAutoLogged;
import frc2713.lib.io.TalonFXIO;
import frc2713.lib.subsystem.MotorSubsystem;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;
import java.util.function.Supplier;

public class Flywheels extends MotorSubsystem<MotorInputsAutoLogged, TalonFXIO> {

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
    // Additional periodic code for Launcher can be added here
  }
}
