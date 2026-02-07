package frc2713.lib.io;

import static edu.wpi.first.units.Units.Revolutions;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import frc2713.lib.drivers.CANDeviceId;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;
import frc2713.lib.util.CTREUtil;
import frc2713.lib.util.LoggedTunableGains;
import frc2713.robot.Robot;
import org.littletonrobotics.junction.AutoLogOutput;

public class TalonFXIO implements MotorIO {
  // Base members
  protected final TalonFX talon;
  protected final TalonFXSubsystemConfig config;
  public final AdvantageScopePathBuilder pb;

  // Control signals
  DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0);
  private final VelocityVoltage velocityVoltageControl = new VelocityVoltage(0.0);
  private final VoltageOut voltageControl = new VoltageOut(0.0);
  private final PositionVoltage positionVoltageControl = new PositionVoltage(0.0);
  private final MotionMagicVoltage motionMagicPositionControl = new MotionMagicVoltage(0.0);
  private final DynamicMotionMagicVoltage dynamicMotionMagicVoltage =
      new DynamicMotionMagicVoltage(0.0, 0.0, 0.0);
  private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0);

  // Status signals
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> currentStatorSignal;
  private final StatusSignal<Current> currentSupplySignal;
  private final StatusSignal<Angle> rawRotorPositionSignal;

  private final BaseStatusSignal[] signals;

  // Tunables for Slot0 and MotionMagic (created when config.tunable == true)
  private LoggedTunableGains tunableGains = null;

  public TalonFXIO(TalonFXSubsystemConfig config) {
    this.talon =
        new TalonFX(config.talonCANID.getDeviceNumber(), new CANBus(config.talonCANID.getBus()));
    this.config = config;

    this.pb = new AdvantageScopePathBuilder(this.config.name);

    // Current limits and ramp rates do not perform well in sim.
    if (Robot.isSimulation()) {
      this.config.fxConfig.CurrentLimits = new CurrentLimitsConfigs();
      this.config.fxConfig.ClosedLoopRamps = new ClosedLoopRampsConfigs();
      this.config.fxConfig.OpenLoopRamps = new OpenLoopRampsConfigs();
    }

    CTREUtil.applyConfiguration(this.talon, this.config.fxConfig);

    positionSignal = talon.getPosition();
    velocitySignal = talon.getVelocity();
    voltageSignal = talon.getMotorVoltage();
    currentStatorSignal = talon.getStatorCurrent();
    currentSupplySignal = talon.getSupplyCurrent();
    rawRotorPositionSignal = talon.getRotorPosition();
    signals =
        new BaseStatusSignal[] {
          positionSignal,
          velocitySignal,
          voltageSignal,
          currentStatorSignal,
          currentSupplySignal,
          rawRotorPositionSignal
        };

    CTREUtil.tryUntilOK(
        () -> BaseStatusSignal.setUpdateFrequencyForAll(50.0, signals), talon.getDeviceID());
    CTREUtil.tryUntilOK(() -> talon.optimizeBusUtilization(), talon.getDeviceID());

    // If this Talon is marked tunable, create dashboard tunables for PID gains
    if (this.config.tunable) {
      tunableGains =
          new LoggedTunableGains(
              this.config.name, this.config.fxConfig.Slot0, this.config.fxConfig.MotionMagic);
    }
  }

  @Override
  public void readInputs(MotorInputs inputs) {
    BaseStatusSignal.refreshAll(signals);
    inputs.position = positionSignal.getValue();
    inputs.velocity = velocitySignal.getValue();
    inputs.appliedVolts = voltageSignal.getValue();
    inputs.currentStatorAmps = currentStatorSignal.getValue();
    inputs.currentSupplyAmps = currentSupplySignal.getValue();
    inputs.rawRotorPosition = rawRotorPositionSignal.getValue();

    // Update PID gains from dashboard if tunable and any value changed
    if (this.config.tunable && tunableGains != null) {
      tunableGains.ifChanged(
          this.hashCode(),
          (Slot0Configs gains, MotionMagicConfigs motionMagic) -> {
            // Apply updated PID/FF gains
            this.config.fxConfig.Slot0 = gains;
            this.config.fxConfig.MotionMagic = motionMagic;
            CTREUtil.applyConfiguration(talon, this.config.fxConfig);
          });
    }
  }

  @Override
  public void setOpenLoopDutyCycle(double dutyCycle) {
    talon.setControl(dutyCycleControl.withOutput(dutyCycle));
  }

  @Override
  public void setPositionSetpoint(Angle setpoint) {
    talon.setControl(positionVoltageControl.withPosition(setpoint));
  }

  @Override
  public void setMotionMagicSetpoint(Angle setpoint, int slot) {
    talon.setControl(motionMagicPositionControl.withPosition(setpoint));
  }

  @Override
  public void setMotionMagicSetpoint(
      Angle setpoint,
      AngularVelocity velocity,
      AngularAcceleration acceleration,
      Velocity<AngularAccelerationUnit> jerk,
      int slot,
      double feedforward) {
    talon.setControl(
        dynamicMotionMagicVoltage
            .withPosition(setpoint)
            .withVelocity(velocity)
            .withAcceleration(acceleration)
            .withJerk(jerk)
            .withFeedForward(feedforward));
  }

  @Override
  public void setNeutralMode(NeutralModeValue mode) {
    config.fxConfig.MotorOutput.NeutralMode = mode;
    CTREUtil.applyConfiguration(talon, config.fxConfig);
  }

  @Override
  public void setVelocitySetpoint(AngularVelocity setpoint, int slot) {
    talon.setControl(velocityVoltageControl.withVelocity(setpoint));
  }

  @Override
  public void setVoltageOutput(Voltage voltage) {
    talon.setControl(voltageControl.withOutput(voltage));
  }

  @Override
  public void setCurrentPositionAsZero() {
    setCurrentPosition(Revolutions.of(0.0));
  }

  @Override
  public void setCurrentPosition(Angle position) {
    talon.setPosition(position);
  }

  @Override
  public void setEnableSoftLimits(boolean forward, boolean reverse) {
    config.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = forward;
    config.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverse;
    CTREUtil.applyConfiguration(talon, config.fxConfig);
  }

  @Override
  public void setEnableHardLimits(boolean forward, boolean reverse) {
    config.fxConfig.HardwareLimitSwitch.ForwardLimitEnable = forward;
    config.fxConfig.HardwareLimitSwitch.ReverseLimitEnable = reverse;
    CTREUtil.applyConfiguration(talon, config.fxConfig);
  }

  @Override
  public void setEnableAutosetPositionValue(boolean forward, boolean reverse) {
    config.fxConfig.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = forward;
    config.fxConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = reverse;
    config.fxConfig.HardwareLimitSwitch.ForwardLimitEnable = forward;
    config.fxConfig.HardwareLimitSwitch.ReverseLimitEnable = reverse;
    CTREUtil.applyConfiguration(talon, config.fxConfig.HardwareLimitSwitch);
  }

  @Override
  public void follow(CANDeviceId leaderId, boolean opposeLeaderDirection) {
    MotorAlignmentValue alignment =
        opposeLeaderDirection ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned;
    CTREUtil.tryUntilOK(
        () -> talon.setControl(new Follower(leaderId.getDeviceNumber(), alignment)),
        this.config.talonCANID.getDeviceNumber());
  }

  @Override
  public void setTorqueCurrentFOC(Current current) {
    talon.setControl(torqueCurrentFOC.withOutput(current));
  }

  @Override
  public void setMotionMagicConfig(MotionMagicConfigs config) {
    this.config.fxConfig.MotionMagic = config;
    CTREUtil.applyConfiguration(talon, this.config.fxConfig.MotionMagic);
  }

  @Override
  public void setVoltageConfig(VoltageConfigs config) {
    CTREUtil.applyConfigurationNonBlocking(talon, config);
  }

  @AutoLogOutput
  public double getClosedLoopError() {
    return talon.getClosedLoopError().getValue();
  }

  public boolean isMagicMotionAtTarget() {
    return talon.getMotionMagicAtTarget(true).getValue();
  }
}
