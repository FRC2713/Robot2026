package frc2713.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc2713.lib.io.MotorIO;
import frc2713.lib.io.MotorInputsAutoLogged;
import frc2713.lib.subsystem.MotorSubsystem;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;

public class IntakeRoller extends MotorSubsystem<MotorInputsAutoLogged, MotorIO> {

  public IntakeRoller(final TalonFXSubsystemConfig config, final MotorIO intakeRollersMotorIO) {
    super(config, new MotorInputsAutoLogged(), intakeRollersMotorIO);
  }

  public Command setIntakeVoltageCommand(Voltage volts) {
    return voltageCommand(() -> volts);
  }

  public Command intake() {
    return setIntakeVoltageCommand(IntakeConstants.Roller.intakeVoltageDesired);
  }

  public Command stop() {
    return setIntakeVoltageCommand(Volts.of(0.0));
  }

  public Command outtake() {
    return setIntakeVoltageCommand(IntakeConstants.Roller.outtakeVoltageDesired);
  }

  @Override
  public void periodic() {
    super.periodic();
    // Additional periodic code for intake rollers can be added here
  }
}
