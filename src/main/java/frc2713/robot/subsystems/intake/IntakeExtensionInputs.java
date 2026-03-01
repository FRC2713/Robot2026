package frc2713.robot.subsystems.intake;

import frc2713.lib.io.MotorInputs;
import frc2713.lib.io.MotorInputsAutoLogged;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class IntakeExtensionInputs extends MotorInputs {
  // Also log leader and follower inputs
  public MotorInputsAutoLogged leader = new MotorInputsAutoLogged();
  public MotorInputsAutoLogged follower = new MotorInputsAutoLogged();
}
