package frc2713.lib.io;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import java.util.function.Supplier;

/**
 * Simulation implementation of CanCoderIO. Sets CANcoder sim state from a supplier before reading,
 * so mechanism sim can drive the encoder values.
 */
public class SimCanCoderIO extends CanCoderIOHardware {

  /** State supplied by the mechanism sim for CANcoder position and velocity. */
  public static class SimCanCoderState {
    public double positionRotations = Double.NaN;
    public double velocityRotationsPerSecond = Double.NaN;
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
    simState.setRawPosition(invertMultiplier * suppliedState.positionRotations);
    simState.setVelocity(invertMultiplier * suppliedState.velocityRotationsPerSecond);

    super.readInputs(inputs);
  }
}
