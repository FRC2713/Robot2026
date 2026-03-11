package frc2713.lib.io;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.function.Supplier;

/**
 * Simulation implementation of CanCoderIO. Sets CANcoder sim state from a supplier before reading,
 * so mechanism sim can drive the encoder values.
 */
public class SimCanCoderIO extends CanCoderIOHardware {

  /** State supplied by the mechanism sim for CANcoder position and velocity. */
  public static class SimCanCoderState {
    public Angle position = null;
    public AngularVelocity velocity = null;
  }

  protected final CANcoderSimState simState;
  protected final Supplier<SimCanCoderState> supplier;

  public SimCanCoderIO(CanCoderConfig config, Supplier<SimCanCoderState> supplier) {
    super(config);
    this.simState = canCoder.getSimState();
    this.supplier = supplier;
    canCoder.setPosition(0.0);
  }

  @Override
  public void readInputs(CanCoderInputs inputs) {
    double invertMultiplier =
        config.config.MagnetSensor.SensorDirection == SensorDirectionValue.CounterClockwise_Positive
            ? +1.0
            : -1.0;
    var suppliedState = supplier.get();

    // Set sim state
    simState.setRawPosition(suppliedState.position.times(invertMultiplier));
    simState.setVelocity(suppliedState.velocity.times(invertMultiplier));

    // Explicitly refresh signals for simulation to ensure they are updated
    super.readInputs(inputs);

    // If signals are still null after refresh in SIM,
    // it might be because the signal hasn't been "received" by the StatusSignal object yet.
    if (inputs.absolutePosition == null) {
      inputs.absolutePosition =
          Rotations.of(suppliedState.position.in(edu.wpi.first.units.Units.Rotations) % 1.0);
    }
    if (inputs.position == null) {
      inputs.position = suppliedState.position;
    }
    if (inputs.velocity == null) {
      inputs.velocity = suppliedState.velocity;
    }
  }
}
