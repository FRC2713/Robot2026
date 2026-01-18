package frc2713.lib.io;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;
import frc2713.lib.util.RobotTime;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.Logger;

public class SimTalonFXIO extends TalonFXIO {
  protected DCMotorSim sim;
  private Notifier simNotifier = null;
  private Time lastUpdateTimestamp = Time.ofBaseUnits(0.0, Seconds);
  private Optional<AngularVelocity> overrideVelocity = Optional.empty();
  private Optional<Angle> overridePosition = Optional.empty();

  // Used to handle mechanisms that wrap.
  private boolean invertVoltage = false;

  protected AtomicReference<Angle> lastPosition =
      new AtomicReference<>((Angle) Angle.ofBaseUnits(0.0, Rotations));
  protected AtomicReference<AngularVelocity> lastVelocity =
      new AtomicReference<>((AngularVelocity) AngularVelocity.ofBaseUnits(0.0, RadiansPerSecond));

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
    super(config);
    this.sim = sim;
    talon.getSimState().Orientation =
        (config.fxConfig.MotorOutput.Inverted == InvertedValue.Clockwise_Positive)
            ? ChassisReference.Clockwise_Positive
            : ChassisReference.CounterClockwise_Positive;
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
    Logger.recordOutput(config.name + "/Sim/setPositionRad", rad);
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
    double simVoltage = addFriction(simState.getMotorVoltage(), 0.25);
    simVoltage = (invertVoltage) ? -simVoltage : simVoltage;
    sim.setInput(simVoltage);
    Logger.recordOutput(config.name + "/Sim/SimulatorVoltage", simVoltage);

    Time timestamp = RobotTime.getTimestamp();
    sim.update(timestamp.minus(lastUpdateTimestamp).in(Seconds));
    lastUpdateTimestamp = timestamp;

    overridePosition.ifPresent(anAngle -> sim.setAngle(anAngle.in(Radians)));

    // Find current state of sim in radians from 0 point
    Angle simPosition = Angle.ofBaseUnits(sim.getAngularPositionRad(), Radians);
    Logger.recordOutput(config.name + "/Sim/SimulatorPositionRadians", simPosition.in(Radians));

    // Mutate rotor position
    Angle rotorPosition = simPosition.div(getSimRatio());
    lastPosition.set(rotorPosition);
    simState.setRawRotorPosition(rotorPosition);
    Logger.recordOutput(config.name + "/Sim/setRawRotorPosition", rotorPosition);

    // Mutate rotor vel
    AngularVelocity rotorVel =
        AngularVelocity.ofBaseUnits(sim.getAngularVelocityRadPerSec(), RadiansPerSecond)
            .div(getSimRatio());
    lastVelocity.set(rotorVel);
    simState.setRotorVelocity(overrideVelocity.isEmpty() ? rotorVel : overrideVelocity.get());
    Logger.recordOutput(
        config.name + "/Sim/SimulatorVelocityRadS", sim.getAngularVelocityRadPerSec());
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
}
