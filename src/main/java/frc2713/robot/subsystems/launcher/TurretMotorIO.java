package frc2713.robot.subsystems.launcher;

import frc2713.lib.io.MotorIO;

/** IO interface for the turret motor with dual encoder support */
public interface TurretMotorIO extends MotorIO {
  /**
   * Reads all turret inputs including both encoder positions.
   *
   * @param inputs The TurretInputs object to populate with sensor data
   */
  void readInputs(TurretInputs inputs);
}
