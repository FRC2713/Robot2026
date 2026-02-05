package frc2713.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Degrees;
import static frc2713.robot.subsystems.launcher.LauncherConstants.Turret.ENCODER_1_TO_TURRET_RATIO;
import static frc2713.robot.subsystems.launcher.LauncherConstants.Turret.SLOPE;

import frc2713.lib.io.MotorInputs;
import frc2713.lib.io.SimTalonFXIO;
import org.littletonrobotics.junction.Logger;

/**
 * Simulation IO implementation for the turret with dual encoder support. Simulates both the TalonFX
 * integrated encoder and the external CANCoder.
 */
public class TurretMotorIOSim extends SimTalonFXIO implements TurretMotorIO {

  public TurretMotorIOSim(TurretSubsystemConfig config) {
    super(config);
  }

  @Override
  public void readInputs(TurretInputs inputs) {
    // Call parent to read standard motor inputs
    super.readInputs(inputs);

    // rawRotorPosition is the motor position. With unitToRotorRatio = 1.0, this equals turret
    // position
    double turretPositionDegrees = inputs.rawRotorPosition.in(Degrees);

    // Encoder 1 is on GEAR_1, so it spins faster than the turret by ENCODER_1_TO_TURRET_RATIO
    double encoder1Degrees = turretPositionDegrees * ENCODER_1_TO_TURRET_RATIO;
    inputs.encoder1PositionDegrees = Degrees.of(encoder1Degrees);

    // Simulate encoder 2: The Vernier algorithm expects diff = turretPosition / SLOPE
    // where diff = e2 - e1, so e2 = e1 + turretPosition / SLOPE
    double encoder2Degrees = encoder1Degrees + (turretPositionDegrees / SLOPE);
    inputs.encoder2PositionDegrees = Degrees.of(encoder2Degrees);

    // Compute using Vernier algorithm
    double vernierComputed = Turret.turretPositionFromEncoders(encoder1Degrees, encoder2Degrees);
    Logger.recordOutput("Turret/vernierComputedDegrees", vernierComputed);
    Logger.recordOutput("Turret/encoder1Degrees", encoder1Degrees);
    Logger.recordOutput("Turret/encoder2Degrees", encoder2Degrees);

    // Use the Vernier computed position
    inputs.computedTurretPositionDegrees = Degrees.of(vernierComputed);
  }

  @Override
  public void readInputs(MotorInputs inputs) {
    // If called with base MotorInputs, just read motor inputs
    super.readInputs(inputs);
  }
}
