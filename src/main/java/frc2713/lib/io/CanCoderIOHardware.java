package frc2713.lib.io;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc2713.lib.util.CTREUtil;

public class CanCoderIOHardware implements CanCoderIO {
  protected final CANcoder canCoder;
  protected final CanCoderConfig config;

  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final BaseStatusSignal[] signals;

  public CanCoderIOHardware(CanCoderConfig config) {
    this.config = config;
    this.canCoder = new CANcoder(config.canId.getDeviceNumber(), new CANBus(config.canId.getBus()));

    CTREUtil.applyConfiguration(canCoder, config.config);

    positionSignal = canCoder.getAbsolutePosition();
    velocitySignal = canCoder.getVelocity();
    signals = new BaseStatusSignal[] {positionSignal, velocitySignal};

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, signals);
    CTREUtil.tryUntilOK(() -> canCoder.optimizeBusUtilization(), canCoder.getDeviceID());
  }

  @Override
  public void readInputs(CanCoderInputs inputs) {
    BaseStatusSignal.refreshAll(signals);
    inputs.absolutePositionRotations = positionSignal.getValue().in(Rotations);
    inputs.velocityRotationsPerSecond = velocitySignal.getValue().in(RotationsPerSecond);
  }
}
