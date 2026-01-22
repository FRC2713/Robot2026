package frc2713.robot.subsystems.launcher;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc2713.lib.io.MotorInputsAutoLogged;
import frc2713.lib.io.TalonFXIO;
import frc2713.lib.subsystem.MotorSubsystem;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;

public class Turret extends MotorSubsystem<MotorInputsAutoLogged, TalonFXIO> {

  public Turret(final TalonFXSubsystemConfig config, final TalonFXIO turretMotorIO) {
    super(config, new MotorInputsAutoLogged(), turretMotorIO);
  }

  public Command setAngle(Supplier<Angle> desiredAngle) {
    return motionMagicSetpointCommand(() -> convertSubsystemPositionToMotorPosition(desiredAngle.get()));
  }

  @Override
  public void periodic() {
    super.periodic();
    // Additional periodic code for turret can be added here`
  }
}
