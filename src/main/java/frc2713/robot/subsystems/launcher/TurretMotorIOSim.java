package frc2713.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Degrees;

import frc2713.lib.io.MotorInputs;
import frc2713.lib.io.SimTalonFXIO;

/**
 * Simulation IO implementation for the turret with dual encoder support. Simulates both the TalonFX
 * integrated encoder and the external CANCoder.
 */
public class TurretMotorIOSim extends SimTalonFXIO implements TurretMotorIO {

  // Simulated encoder offsets to differentiate the two encoders
  private static final double ENCODER_2_OFFSET_DEGREES = 15.0;

  public TurretMotorIOSim(TurretSubsystemConfig config) {
    super(config);
  }

  @Override
  public void readInputs(TurretInputs inputs) {
    // Call parent to read standard motor inputs
    super.readInputs(inputs);

    // Simulate encoder 1 (TalonFX integrated encoder) - use raw rotor position
    double talonEncoderDegrees = inputs.rawRotorPosition.in(Degrees);
    inputs.encoder1PositionDegrees = Degrees.of(talonEncoderDegrees);

    // Simulate encoder 2 (external CANCoder) - slightly different gearing creates offset
    // In reality, the CANCoder would be on a different gear with different tooth count
    double canCoderDegrees = talonEncoderDegrees + ENCODER_2_OFFSET_DEGREES;
    inputs.encoder2PositionDegrees = Degrees.of(canCoderDegrees);

    // Compute turret position from both encoders using the Vernier algorithm
    double computedPosition =
        Turret.turretPositionFromEncoders(
            inputs.encoder1PositionDegrees.in(Degrees), inputs.encoder2PositionDegrees.in(Degrees));
    inputs.computedTurretPositionDegrees = Degrees.of(computedPosition);
  }

  @Override
  public void readInputs(MotorInputs inputs) {
    // If called with base MotorInputs, just read motor inputs
    super.readInputs(inputs);
  }
}
