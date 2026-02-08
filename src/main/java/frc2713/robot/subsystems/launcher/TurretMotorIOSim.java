package frc2713.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Degrees;
import static frc2713.robot.subsystems.launcher.LauncherConstants.Turret.ENCODER_1_TO_TURRET_RATIO;
import static frc2713.robot.subsystems.launcher.LauncherConstants.Turret.SLOPE;

import frc2713.lib.io.MotorInputs;
import frc2713.lib.io.SimTalonFXIO;

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

    // Gear train:
    // - Encoder 1 is on the motor shaft (GEAR_0) - directly measures motor position
    // - GEAR_1 meshes with GEAR_0
    // - Encoder 2 (CANCoder) is on GEAR_2
    // - The turret position is computed from both encoders using the Vernier algorithm

    // Encoder 1 = motor position (rawRotorPosition) since it's on the motor shaft
    double encoder1Degrees = inputs.rawRotorPosition.in(Degrees);
    inputs.encoder1PositionDegrees = encoder1Degrees;

    // Calculate the turret position from encoder 1
    // Encoder 1 spins ENCODER_1_TO_TURRET_RATIO times per turret rotation
    // turretPosition = encoder1 / ENCODER_1_TO_TURRET_RATIO
    double turretPositionDegrees = encoder1Degrees / ENCODER_1_TO_TURRET_RATIO;

    // Simulate encoder 2: The Vernier algorithm expects diff = turretPosition / SLOPE
    // where diff = e2 - e1, so e2 = e1 + turretPosition / SLOPE
    double encoder2Degrees = encoder1Degrees + (turretPositionDegrees / SLOPE);
    inputs.encoder2PositionDegrees = encoder2Degrees;

    // Compute using Vernier algorithm (should match turretPositionDegrees)
    double vernierComputed = Turret.turretPositionFromEncoders(encoder1Degrees, encoder2Degrees);

    // Use the Vernier computed position
    inputs.computedTurretPositionDegrees = vernierComputed;
  }

  @Override
  public void readInputs(MotorInputs inputs) {
    // If called with base MotorInputs, just read motor inputs
    super.readInputs(inputs);
  }
}
