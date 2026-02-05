package frc2713.lib.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;
import frc2713.lib.util.LoggedTunableNumber;
import frc2713.lib.util.RobotTime;
import java.util.concurrent.atomic.AtomicReference;

public class SimTalonFXIO extends TalonFXIO {
  protected DCMotorSim sim;
  private Notifier simNotifier = null;
  private Time lastUpdateTimestamp = Seconds.of(0.0);

  // Tunable PID gains for simulation closed-loop control
  private static final LoggedTunableNumber simPidKp =
      new LoggedTunableNumber("SimTalonFX/PID/kP", 12.0);
  private static final LoggedTunableNumber simPidKi =
      new LoggedTunableNumber("SimTalonFX/PID/kI", 0.0);
  private static final LoggedTunableNumber simPidKd =
      new LoggedTunableNumber("SimTalonFX/PID/kD", 3.0);
  private static final LoggedTunableNumber simPidIZone =
      new LoggedTunableNumber("SimTalonFX/PID/iZone", 2.0);

  // Used to handle mechanisms that wrap.
  private boolean invertVoltage = false;

  protected AtomicReference<Angle> lastPosition = new AtomicReference<>((Angle) Rotations.of(0.0));
  protected AtomicReference<AngularVelocity> lastVelocity =
      new AtomicReference<>((AngularVelocity) RadiansPerSecond.of(0.0));

  protected double getSimRatio() {
    return config.unitToRotorRatio;
  }

  public SimTalonFXIO(TalonFXSubsystemConfig config) {
    // the TalonFX in the parent class will store and update the motor charecteristics in simulation
    // the DCMotorSim has the plant model that lets up update the motor state while running in
    // simulation
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

  protected double getPlantModelInput(double frictionVoltage) {
    double motorVoltage = talon.getSimState().getMotorVoltage();

    // Apply static friction
    if (Math.abs(motorVoltage) < frictionVoltage) {
      motorVoltage = 0.0;
    } else if (motorVoltage > 0.0) {
      motorVoltage -= frictionVoltage;
    } else {
      motorVoltage += frictionVoltage;
    }

    // Apply motor inversion
    return invertVoltage ? -motorVoltage : motorVoltage;
  }

  public void setInvertVoltage(boolean invertVoltage) {
    this.invertVoltage = invertVoltage;
  }

  protected void updateSimState() {
    var motorState = talon.getSimState();
    motorState.setSupplyVoltage((invertVoltage ? -1 : 1) * RobotController.getBatteryVoltage());

    // Use our closed-loop voltage if we have a setpoint, otherwise use Phoenix voltage
    double voltage;
    if (positionSetpointRotations != null) {
      voltage = getClosedLoopVoltage();
    } else {
      voltage = getPlantModelInput(0.25);
    }
    sim.setInputVoltage(voltage);

    Time timestamp = RobotTime.getTimestamp();
    sim.update(timestamp.minus(lastUpdateTimestamp).in(Seconds));
    lastUpdateTimestamp = timestamp;

    double simPositionRad = sim.getAngularPositionRad(); // sim in rads
    double simVelocityRadPerSec = sim.getAngularVelocityRadPerSec();

    double motorRotations = simPositionRad / (2.0 * Math.PI); // ctre expects rotations
    double motorRotPerSec = simVelocityRadPerSec / (2.0 * Math.PI);
    motorState.setRawRotorPosition(motorRotations);
    motorState.setRotorVelocity(motorRotPerSec);
  }

  @Override
  public void readInputs(MotorInputs inputs) {
    // Note that in SIM we dont call BaseStatusSignal.refreshAll(signals) to avoid CAN errors
    // so the inputs have to come from the sim. There might still be some CAN errors at init.

    double simPositionRad = sim.getAngularPositionRad();
    double simVelocityRadPerSec = sim.getAngularVelocityRadPerSec();

    // Convert from radians to rotations (mechanism position, not rotor)
    double mechanismRotations = simPositionRad / (2.0 * Math.PI) * config.unitToRotorRatio;
    double mechanismRotPerSec = simVelocityRadPerSec / (2.0 * Math.PI) * config.unitToRotorRatio;

    inputs.position = Rotations.of(mechanismRotations);
    inputs.velocity = RotationsPerSecond.of(mechanismRotPerSec);
    inputs.appliedVolts = Volts.of(sim.getInputVoltage());
    inputs.currentStatorAmps = Amps.of(sim.getCurrentDrawAmps());
    inputs.currentSupplyAmps = Amps.of(sim.getCurrentDrawAmps());
    inputs.rawRotorPosition = Rotations.of(simPositionRad / (2.0 * Math.PI));
  }

  // Setpoint for simulated closed-loop control (in motor rotations)
  private volatile Double positionSetpointRotations = null;
  private double errorIntegral = 0.0;
  private double lastError = 0.0;
  private long lastPidTimeNanos = 0;

  @Override
  public void setMotionMagicSetpoint(Angle setpoint, int slot) {
    // Store setpoint for our sim closed-loop control (in motor rotations)
    double newSetpoint = setpoint.in(Rotations) / config.unitToRotorRatio;
    if (positionSetpointRotations == null
        || Math.abs(newSetpoint - positionSetpointRotations) > 0.001) {
      // Reset integral and derivative state on new setpoint
      errorIntegral = 0.0;
      lastError = 0.0;
      lastPidTimeNanos = System.nanoTime();
    }
    positionSetpointRotations = newSetpoint;
    super.setMotionMagicSetpoint(setpoint, slot);
  }

  @Override
  public void setPositionSetpoint(Angle setpoint) {
    // Store setpoint for our sim closed-loop control (in motor rotations)
    double newSetpoint = setpoint.in(Rotations) / config.unitToRotorRatio;
    if (positionSetpointRotations == null
        || Math.abs(newSetpoint - positionSetpointRotations) > 0.001) {
      // Reset integral and derivative state on new setpoint
      errorIntegral = 0.0;
      lastError = 0.0;
      lastPidTimeNanos = System.nanoTime();
    }
    positionSetpointRotations = newSetpoint;
    super.setPositionSetpoint(setpoint);
  }

  /**
   * Calculate voltage for closed-loop position control in simulation. Phoenix 6's closed-loop sim
   * doesn't generate proper voltage with DCMotorSim, so we implement our own PID controller.
   */
  protected double getClosedLoopVoltage() {
    if (positionSetpointRotations == null) {
      return 0.0;
    }

    // Reset PID state if any gains changed
    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> {
          errorIntegral = 0.0;
          lastError = 0.0;
        },
        simPidKp,
        simPidKi,
        simPidKd,
        simPidIZone);

    long currentTimeNanos = System.nanoTime();
    double dt = (currentTimeNanos - lastPidTimeNanos) / 1e9; // Convert to seconds
    if (dt <= 0 || dt > 0.1) {
      dt = 0.005; // Default to 5ms if invalid
    }
    lastPidTimeNanos = currentTimeNanos;

    double currentPositionRotations = sim.getAngularPositionRad() / (2.0 * Math.PI);
    double errorRotations = positionSetpointRotations - currentPositionRotations;

    // PID gains from tunable constants
    double kP = simPidKp.get();
    double kI = simPidKi.get();
    double kD = simPidKd.get();
    double iZone = simPidIZone.get();

    // Integrate error (with anti-windup)
    errorIntegral += errorRotations * dt;
    errorIntegral = Math.max(-iZone, Math.min(iZone, errorIntegral)); // Clamp integral

    // Derivative of error
    double errorDerivative = (errorRotations - lastError) / dt;
    lastError = errorRotations;

    // Calculate voltage
    double voltage = (kP * errorRotations) + (kI * errorIntegral) + (kD * errorDerivative);

    // Clamp to battery voltage
    double maxVoltage = RobotController.getBatteryVoltage();
    voltage = Math.max(-maxVoltage, Math.min(maxVoltage, voltage));

    return invertVoltage ? -voltage : voltage;
  }
}
