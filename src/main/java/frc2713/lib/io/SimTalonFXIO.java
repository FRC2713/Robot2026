package frc2713.lib.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc2713.lib.drivers.CANDeviceId;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;
import frc2713.lib.util.CTREUtil;
import frc2713.lib.util.RobotTime;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.Logger;

public class SimTalonFXIO implements MotorIO {
  protected DCMotorSim sim;
  private Notifier simNotifier = null;
  private Time lastUpdateTimestamp = Seconds.of(0.0);
  private Optional<AngularVelocity> overrideVelocity = Optional.empty();
  private Optional<Angle> overridePosition = Optional.empty();

  protected final TalonFX talon;
  protected final TalonFXSubsystemConfig config;
  public final AdvantageScopePathBuilder pb;

  // Used to handle mechanisms that wrap.
  private boolean invertVoltage = false;

  protected AtomicReference<Angle> lastPosition = new AtomicReference<>((Angle) Rotations.of(0.0));
  protected AtomicReference<AngularVelocity> lastVelocity =
      new AtomicReference<>((AngularVelocity) RadiansPerSecond.of(0.0));

  protected double getSimRatio() {
    return config.unitToRotorRatio;
  }

  public SimTalonFXIO(TalonFXSubsystemConfig config) {
    this(
        config,
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), config.momentOfInertia, 1.0 / config.unitToRotorRatio),
            DCMotor.getKrakenX60Foc(1),
            0.001,
            0.001));
  }

  public SimTalonFXIO(TalonFXSubsystemConfig config, DCMotorSim sim) {
    this.config = config;
    this.sim = sim;
    this.pb = new AdvantageScopePathBuilder(this.config.name);
    this.talon = new TalonFX(config.talonCANID.getDeviceNumber(), config.talonCANID.getBus());

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    simNotifier =
        new Notifier(
            () -> {
              updateSimState();
            });
    simNotifier.startPeriodic(0.005);
  }

  // Need to use rad of the mechanism itself.
  public void setPositionRad(double rad) {
    sim.setAngle(
        (config.fxConfig.MotorOutput.Inverted == InvertedValue.Clockwise_Positive ? -1.0 : 1.0)
            * rad);
    Logger.recordOutput(pb.makePath("Sim", "setPositionRad"), rad);
  }

  protected double addFriction(double motorVoltage, double frictionVoltage) {
    if (Math.abs(motorVoltage) < frictionVoltage) {
      motorVoltage = 0.0;
    } else if (motorVoltage > 0.0) {
      motorVoltage -= frictionVoltage;
    } else {
      motorVoltage += frictionVoltage;
    }
    return motorVoltage;
  }

  public void setInvertVoltage(boolean invertVoltage) {
    this.invertVoltage = invertVoltage;
  }

  protected void updateSimState() {
    var simState = talon.getSimState();

    // double simVoltage = addFriction(simState.getMotorVoltage(), 0.25);
    // simVoltage = (invertVoltage) ? -simVoltage : simVoltage;
    // simState.setSupplyVoltage(simVoltage);
    // sim.setInput(simVoltage);
    // Logger.recordOutput(pb.makePath("Sim", "SimulatorVoltage"), simVoltage);

    Time timestamp = RobotTime.getTimestamp();
    sim.update(timestamp.minus(lastUpdateTimestamp).in(Seconds));
    lastUpdateTimestamp = timestamp;

    overridePosition.ifPresent(anAngle -> sim.setAngle(anAngle.in(Radians)));

    // Find current state of sim in radians from 0 point
    Angle simPosition = Radians.of(sim.getAngularPositionRad());
    Logger.recordOutput(pb.makePath("Sim", "SimulatorPositionRadians"), simPosition.in(Radians));

    // Mutate rotor position
    Angle rotorPosition = simPosition.div(getSimRatio());
    lastPosition.set(rotorPosition);
    simState.setRawRotorPosition(rotorPosition);
    Logger.recordOutput(pb.makePath("Sim", "setRawRotorPosition"), rotorPosition);

    // Mutate rotor vel
    AngularVelocity rotorVel =
        RadiansPerSecond.of(sim.getAngularVelocityRadPerSec()).div(getSimRatio());
    lastVelocity.set(rotorVel);
    simState.setRotorVelocity(overrideVelocity.isEmpty() ? rotorVel : overrideVelocity.get());
    Logger.recordOutput(
        pb.makePath("Sim", "SimulatorVelocityRadS"), sim.getAngularVelocityRadPerSec());
  }

  public void overrideVelocity(Optional<AngularVelocity> rps) {
    overrideVelocity = rps;
  }

  public void overridePosition(Optional<Angle> pos) {
    overridePosition = pos;
  }

  public AngularVelocity getIntendedVelocity() {
    return lastVelocity.get();
  }

  @Override
  public void readInputs(MotorInputs inputs) {
    inputs.position = sim.getAngularPosition();
    inputs.velocity = sim.getAngularVelocity();
    inputs.currentStatorAmps = Amps.of(0.0);
    inputs.currentSupplyAmps = Amps.of(0.0);
    inputs.rawRotorPosition = Rotations.of(0.0);
  }

  @Override
  public void setOpenLoopDutyCycle(double dutyCycle) {
    // TODO Auto-generated method stub
    sim.setInputVoltage(dutyCycle * 12);
  }

  @Override
  public void setPositionSetpoint(Angle setpoint) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setPositionSetpoint'");
  }

  @Override
  public void setMotionMagicSetpoint(
      Angle setpoint,
      AngularVelocity velocity,
      AngularAcceleration acceleration,
      Velocity<AngularAccelerationUnit> jerk,
      int slot,
      double feedforward) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setMotionMagicSetpoint'");
  }

  @Override
  public void setNeutralMode(NeutralModeValue mode) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setNeutralMode'");
  }

  @Override
  public void setVelocitySetpoint(AngularVelocity unitsPerSecond, int slot) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setVelocitySetpoint'");
  }

  @Override
  public void setVoltageOutput(Voltage voltage) {
    // TODO Auto-generated method stub
    System.out.println("Being called " + voltage);
    sim.setInputVoltage(voltage.in(Volts));
  }

  @Override
  public void setCurrentPositionAsZero() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setCurrentPositionAsZero'");
  }

  @Override
  public void setCurrentPosition(Angle position) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setCurrentPosition'");
  }

  @Override
  public void setEnableSoftLimits(boolean forward, boolean reverse) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setEnableSoftLimits'");
  }

  @Override
  public void setEnableHardLimits(boolean forward, boolean reverse) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setEnableHardLimits'");
  }

  @Override
  public void setEnableAutosetPositionValue(boolean forward, boolean reverse) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setEnableAutosetPositionValue'");
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
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setTorqueCurrentFOC'");
  }

  @Override
  public void setMotionMagicConfig(MotionMagicConfigs config) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setMotionMagicConfig'");
  }

  @Override
  public void setVoltageConfig(VoltageConfigs config) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setVoltageConfig'");
  }
}
