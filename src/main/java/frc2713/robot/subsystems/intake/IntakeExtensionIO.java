package frc2713.robot.subsystems.intake;

import frc2713.lib.io.MotorIO;

public interface IntakeExtensionIO extends MotorIO {
  default void readInputs(IntakeExtensionInputs inputs) {}
}
