package frc2713.robot.subsystems.intake.intakeExtensionIO;

import frc2713.lib.io.MotorInputs;
import frc2713.lib.io.MotorInputsAutoLogged;

import static edu.wpi.first.units.Units.InchesPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.LinearVelocity;

@AutoLog
public class IntakeExtensionInputs extends MotorInputs {
  // Also log leader and follower inputs
  public MotorInputsAutoLogged leader = new MotorInputsAutoLogged();
  public MotorInputsAutoLogged follower = new MotorInputsAutoLogged();
}
