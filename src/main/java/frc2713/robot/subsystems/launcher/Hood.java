package frc2713.robot.subsystems.launcher;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc2713.lib.io.MotorInputsAutoLogged;
import frc2713.lib.io.TalonFXIO;
import frc2713.lib.subsystem.MotorSubsystem;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;

public class Hood extends MotorSubsystem<MotorInputsAutoLogged, TalonFXIO> {

  public Hood(final TalonFXSubsystemConfig config, final TalonFXIO launcherMotorIO) {
    super(config, new MotorInputsAutoLogged(), launcherMotorIO);
  }

  public Command setAngleCommand(Supplier<Angle> desiredAngle) {
    return motionMagicSetpointCommand(() -> convertSubsystemPositionToMotorPosition(desiredAngle.get()));
  }

  public Command retract() {
    return setAngleCommand(() -> LauncherConstants.Hood.retractedPosition);
  }
  
  @Override
  public void periodic() {
    super.periodic();
    // Additional periodic code for Hood can be added here
  }
}
