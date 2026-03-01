package frc2713.robot.subsystems.intake;

import frc2713.lib.io.MotorIO;
import frc2713.lib.io.MotorInputs;

public interface IntakeExtensionIO extends MotorIO {
  default void readInputs(MotorInputs inputs) {}
}
