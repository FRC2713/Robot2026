package frc2713.robot.subsystems.intake.intakeExtensionIO;

import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.DifferentialMechanism;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.*;
import frc2713.lib.drivers.CANDeviceId;
import frc2713.lib.io.AdvantageScopePathBuilder;
import frc2713.lib.subsystem.DifferentialSubsystemConfig;
import frc2713.lib.util.CTREUtil;
import frc2713.lib.util.LoggedTunableGains;
import frc2713.robot.Robot;

public class IntakeExtensionIOTalonFX implements IntakeExtensionIO {

  // Base members
  protected final DifferentialMechanism<TalonFX> diffMech;
  protected final TalonFX leaderMotor;
  protected final TalonFX followerMotor;
  protected final DifferentialSubsystemConfig config;
  public final AdvantageScopePathBuilder pb;

  // Control signals for average axis
  private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0);
  private final PositionDutyCycle positionDutyCycleControl = new PositionDutyCycle(0.0);
  private final VelocityVoltage velocityVoltageControl = new VelocityVoltage(0.0);
  private final VelocityTorqueCurrentFOC velocityTorqueFOCControl =
      new VelocityTorqueCurrentFOC(0.0);
  private final VoltageOut voltageControl = new VoltageOut(0.0);
  private final PositionVoltage positionVoltageControl = new PositionVoltage(0.0);
  private final MotionMagicVoltage motionMagicPositionControl = new MotionMagicVoltage(0.0);
  private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0);

  // Status signals from average axis (differential mechanism provides these)
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> currentStatorSignal;
  private final StatusSignal<Current> currentSupplySignal;
  private final StatusSignal<Current> currentTorqueSignal;
  private final StatusSignal<Angle> rawRotorPositionSignal;
  private final StatusSignal<Double> closedLoopErrorSignal;
  private final StatusSignal<Boolean> motionMagicAtTargetSignal;
  private final BaseStatusSignal[] signals;

  // Tunables for differential gains (created when config.tunable == true)
  protected LoggedTunableGains tunableGains = null;

  public IntakeExtensionIOTalonFX(DifferentialSubsystemConfig config) {
    this.config = config;
    this.pb = new AdvantageScopePathBuilder(config.name);

    // Current limits and ramp rates do not perform well in sim.
    if (Robot.isSimulation()) {
      config.leaderConfig.CurrentLimits = new com.ctre.phoenix6.configs.CurrentLimitsConfigs();
      config.leaderConfig.ClosedLoopRamps = new com.ctre.phoenix6.configs.ClosedLoopRampsConfigs();
      config.leaderConfig.OpenLoopRamps = new com.ctre.phoenix6.configs.OpenLoopRampsConfigs();

      config.followerConfig.CurrentLimits = new com.ctre.phoenix6.configs.CurrentLimitsConfigs();
      config.followerConfig.ClosedLoopRamps =
          new com.ctre.phoenix6.configs.ClosedLoopRampsConfigs();
      config.followerConfig.OpenLoopRamps = new com.ctre.phoenix6.configs.OpenLoopRampsConfigs();
    }

    // Create the differential mechanism using the constants from config
    this.diffMech = new DifferentialMechanism<TalonFX>(TalonFX::new, config.differentialConstants);
    diffMech.configNeutralMode(NeutralModeValue.Coast);

    // Get references to the individual motors from the differential mechanism
    this.leaderMotor = diffMech.getLeader();
    this.followerMotor = diffMech.getFollower();

    // Set up status signals from the differential mechanism's average axis
    positionSignal = diffMech.getAveragePosition();
    velocitySignal = diffMech.getAverageVelocity();
    // For voltage and current, we'll average the leader and follower manually since no direct
    // method exists
    voltageSignal = leaderMotor.getMotorVoltage();
    currentStatorSignal = leaderMotor.getStatorCurrent();
    currentSupplySignal = leaderMotor.getSupplyCurrent();
    currentTorqueSignal = leaderMotor.getTorqueCurrent();
    // Use leader motor for individual signals
    rawRotorPositionSignal = leaderMotor.getRotorPosition();
    closedLoopErrorSignal = leaderMotor.getClosedLoopError();
    motionMagicAtTargetSignal = leaderMotor.getMotionMagicAtTarget();

    signals =
        new BaseStatusSignal[] {
          positionSignal,
          velocitySignal,
          voltageSignal,
          currentStatorSignal,
          currentSupplySignal,
          currentTorqueSignal,
          rawRotorPositionSignal,
          closedLoopErrorSignal,
          motionMagicAtTargetSignal,
        };

    CTREUtil.tryUntilOK(
        () -> BaseStatusSignal.setUpdateFrequencyForAll(50.0, signals),
        config.leaderCANID.getDeviceNumber());
    CTREUtil.tryUntilOK(
        () -> leaderMotor.optimizeBusUtilization(), config.leaderCANID.getDeviceNumber());
    CTREUtil.tryUntilOK(
        () -> followerMotor.optimizeBusUtilization(), config.followerCANID.getDeviceNumber());

    // If this differential is marked tunable, create dashboard tunables for PID gains
    if (config.tunable) {
      tunableGains =
          new LoggedTunableGains(config.name, config.averageGains, config.leaderConfig.MotionMagic);
    }
  }

  @Override
  public void readInputs(IntakeExtensionInputs inputs) {
    BaseStatusSignal.refreshAll(signals);
    inputs.position = positionSignal.getValue();
    inputs.velocity = velocitySignal.getValue();
    inputs.appliedVolts = voltageSignal.getValue();
    inputs.currentStatorAmps = currentStatorSignal.getValue();
    inputs.currentSupplyAmps = currentSupplySignal.getValue();
    inputs.currentTorqueAmps = currentTorqueSignal.getValue();
    inputs.rawRotorPosition = rawRotorPositionSignal.getValue();
    inputs.closedLoopError = closedLoopErrorSignal.getValue();
    inputs.isMotionMagicAtTarget = motionMagicAtTargetSignal.getValue();

    inputs.leader.position = leaderMotor.getPosition().getValue();
    inputs.leader.velocity = leaderMotor.getVelocity().getValue();
    inputs.leader.appliedVolts = leaderMotor.getMotorVoltage().getValue();
    inputs.leader.currentStatorAmps = leaderMotor.getStatorCurrent().getValue();
    inputs.leader.currentSupplyAmps = leaderMotor.getSupplyCurrent().getValue();
    inputs.leader.currentTorqueAmps = leaderMotor.getTorqueCurrent().getValue();
    inputs.leader.rawRotorPosition = leaderMotor.getRotorPosition().getValue();
    inputs.leader.closedLoopError = leaderMotor.getClosedLoopError().getValue();
    inputs.leader.isMotionMagicAtTarget = leaderMotor.getMotionMagicAtTarget().getValue();

    inputs.follower.position = followerMotor.getPosition().getValue();
    inputs.follower.velocity = followerMotor.getVelocity().getValue();
    inputs.follower.appliedVolts = followerMotor.getMotorVoltage().getValue();
    inputs.follower.currentStatorAmps = followerMotor.getStatorCurrent().getValue();
    inputs.follower.currentSupplyAmps = followerMotor.getSupplyCurrent().getValue();
    inputs.follower.currentTorqueAmps = followerMotor.getTorqueCurrent().getValue();
    inputs.follower.rawRotorPosition = followerMotor.getRotorPosition().getValue();
    inputs.follower.closedLoopError = followerMotor.getClosedLoopError().getValue();
    inputs.follower.isMotionMagicAtTarget = followerMotor.getMotionMagicAtTarget().getValue();

    // Update PID gains from dashboard if tunable and any value changed
    if (config.tunable && tunableGains != null) {
      tunableGains.ifChanged(
          this.hashCode(),
          (com.ctre.phoenix6.configs.Slot0Configs gains, MotionMagicConfigs motionMagic) -> {
            // Apply updated PID/FF gains to both leader and follower
            config.averageGains = gains;
            config.leaderConfig.Slot0 = gains;
            config.leaderConfig.MotionMagic = motionMagic;
            CTREUtil.applyConfiguration(leaderMotor, config.leaderConfig);
            if (!config.followerUsesCommonLeaderConfigs) {
              config.followerConfig.Slot0 = gains;
              CTREUtil.applyConfiguration(followerMotor, config.followerConfig);
            }
          });
    }
  }

  @Override
  public void setOpenLoopDutyCycle(double dutyCycle) {
    // For differential mechanism, we control the average axis
    diffMech.setControl(
        dutyCycleControl.withOutput(dutyCycle),
        positionDutyCycleControl.withPosition(Revolutions.of(0)));
  }

  @Override
  public void setPositionSetpoint(Angle setpoint) {
    diffMech.setControl(
        positionVoltageControl.withPosition(setpoint),
        positionVoltageControl.withPosition(Revolutions.of(0)));
  }

  @Override
  public void setMotionMagicSetpoint(Angle setpoint, int slot) {
    diffMech.setControl(
        motionMagicPositionControl.withPosition(setpoint).withSlot(slot),
        positionVoltageControl.withPosition(Revolutions.of(0)));
  }

  @Override
  public void setMotionMagicSetpoint(
      Angle setpoint,
      AngularVelocity velocity,
      AngularAcceleration acceleration,
      Velocity<AngularAccelerationUnit> jerk,
      int slot,
      double feedforward) {
    // For differential, we need to control average and difference separately
    // For now, use simple motion magic on average axis with zero difference
    diffMech.setControl(
        motionMagicPositionControl
            .withPosition(setpoint)
            .withSlot(slot)
            .withFeedForward(feedforward),
        positionVoltageControl.withPosition(Revolutions.of(0)));
  }

  @Override
  public void setNeutralMode(NeutralModeValue mode) {
    config.leaderConfig.MotorOutput.NeutralMode = mode;
    config.followerConfig.MotorOutput.NeutralMode = mode;
    CTREUtil.applyConfiguration(leaderMotor, config.leaderConfig);
    CTREUtil.applyConfiguration(followerMotor, config.followerConfig);
  }

  @Override
  public void setVelocitySetpoint(AngularVelocity setpoint, int slot) {
    if (config.useFOC)
      diffMech.setControl(
          velocityTorqueFOCControl.withVelocity(setpoint).withSlot(slot),
          new PositionTorqueCurrentFOC(Revolutions.of(0)));
    else
      diffMech.setControl(
          velocityVoltageControl.withVelocity(setpoint).withSlot(slot),
          positionVoltageControl.withPosition(Revolutions.of(0)));
  }

  @Override
  public void setVoltageOutput(Voltage voltage) {
    diffMech.setControl(voltageControl.withOutput(voltage), voltageControl.withOutput(Volts.of(0)));
  }

  @Override
  public void setCurrentPositionAsZero() {
    setCurrentPosition(Revolutions.of(0.0));
  }

  @Override
  public void setCurrentPosition(Angle position) {
    leaderMotor.setPosition(position);
    followerMotor.setPosition(position);
  }

  @Override
  public void setEnableSoftLimits(boolean forward, boolean reverse) {
    config.leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = forward;
    config.leaderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverse;
    config.followerConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = forward;
    config.followerConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverse;
    CTREUtil.applyConfiguration(leaderMotor, config.leaderConfig);
    CTREUtil.applyConfiguration(followerMotor, config.followerConfig);
  }

  @Override
  public void setEnableHardLimits(boolean forward, boolean reverse) {
    config.leaderConfig.HardwareLimitSwitch.ForwardLimitEnable = forward;
    config.leaderConfig.HardwareLimitSwitch.ReverseLimitEnable = reverse;
    config.followerConfig.HardwareLimitSwitch.ForwardLimitEnable = forward;
    config.followerConfig.HardwareLimitSwitch.ReverseLimitEnable = reverse;
    CTREUtil.applyConfiguration(leaderMotor, config.leaderConfig);
    CTREUtil.applyConfiguration(followerMotor, config.followerConfig);
  }

  @Override
  public void setEnableAutosetPositionValue(boolean forward, boolean reverse) {
    config.leaderConfig.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = forward;
    config.leaderConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = reverse;
    config.leaderConfig.HardwareLimitSwitch.ForwardLimitEnable = forward;
    config.leaderConfig.HardwareLimitSwitch.ReverseLimitEnable = reverse;

    config.followerConfig.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = forward;
    config.followerConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = reverse;
    config.followerConfig.HardwareLimitSwitch.ForwardLimitEnable = forward;
    config.followerConfig.HardwareLimitSwitch.ReverseLimitEnable = reverse;

    CTREUtil.applyConfiguration(leaderMotor, config.leaderConfig);
    CTREUtil.applyConfiguration(followerMotor, config.followerConfig);
  }

  @Override
  public void follow(CANDeviceId leaderId, boolean opposeLeaderDirection) {
    // For differential mechanisms, following isn't typical as both motors work together
    // This method is implemented for interface compliance but may not be used
    System.err.println(
        "Warning: follow() called on differential mechanism - this may not be the intended behavior");
  }

  @Override
  public void setTorqueCurrentFOC(Current current) {
    diffMech.setControl(
        torqueCurrentFOC.withOutput(current), torqueCurrentFOC.withOutput(current.times(0)));
  }

  @Override
  public void setMotionMagicConfig(MotionMagicConfigs config) {
    this.config.leaderConfig.MotionMagic = config;
    CTREUtil.applyConfiguration(leaderMotor, this.config.leaderConfig.MotionMagic);
  }

  @Override
  public void setVoltageConfig(VoltageConfigs config) {
    CTREUtil.applyConfigurationNonBlocking(leaderMotor, config);
    CTREUtil.applyConfigurationNonBlocking(followerMotor, config);
  }
}
