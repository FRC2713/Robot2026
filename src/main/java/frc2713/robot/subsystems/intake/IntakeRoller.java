package frc2713.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc2713.lib.logging.PeriodicTimingLogger;
import frc2713.lib.logging.TimeLogged;
import frc2713.lib.io.MotorIO;
import frc2713.lib.io.MotorInputsAutoLogged;
import frc2713.lib.subsystem.MotorFollowerSubsystem;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;

public class IntakeRoller extends MotorFollowerSubsystem<MotorInputsAutoLogged, MotorIO> {

  public IntakeRoller(
      final TalonFXSubsystemConfig leaderConfig,
      final TalonFXSubsystemConfig followerConfig,
      final MotorIO leaderMotorIO,
      final MotorIO followerMotorIO) {
    super(
        leaderConfig.name,
        leaderConfig,
        followerConfig,
        new MotorInputsAutoLogged(),
        new MotorInputsAutoLogged(),
        leaderMotorIO,
        followerMotorIO);
  }

  public Command intake() {
    return voltageCommand(IntakeConstants.Roller.intakeVoltageDesired);
  }

  public Command stop() {
    return voltageCommand(() -> Volts.of(0.0));
  }

  public Command outtake() {
    return voltageCommand(() -> IntakeConstants.Roller.outtakeVoltageDesired);
  }

  @Override
  @TimeLogged("Performance/SubsystemPeriodic/IntakeRoller")
  public void periodic() {
    try (var ignored = PeriodicTimingLogger.time(this)) {
      super.periodic();
      // Additional periodic code for intake rollers can be added here
    }
  }
}
