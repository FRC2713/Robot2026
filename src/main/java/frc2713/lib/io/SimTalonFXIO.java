package frc2713.lib.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import frc2713.lib.util.RobotTime;
import java.util.concurrent.atomic.AtomicReference;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SimTalonFXIO extends TalonFXIO {
  protected DCMotorSim sim;
  private Notifier simNotifier = null;
  private Time lastUpdateTimestamp = Seconds.of(0.0);

  // Used to handle mechanisms that wrap.
  private boolean invertVoltage = false;

  protected AtomicReference<Angle> lastPosition = new AtomicReference<>((Angle) Rotations.of(0.0));
  protected AtomicReference<AngularVelocity> lastVelocity =
      new AtomicReference<>((AngularVelocity) RadiansPerSecond.of(0.0));
  protected double lastClosedLoopError = 0.0;
  protected boolean lastCheckedMMAtTarget = false;

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

    // Apply static friction and brake mode
    if (Math.abs(motorVoltage) < frictionVoltage) {
      if (config.fxConfig.MotorOutput.NeutralMode == NeutralModeValue.Brake) {
        double currentVelocity = sim.getAngularVelocityRadPerSec();
        double brakeVoltage = 6.0; // Voltage to apply for braking (tune as needed)

        if (Math.abs(currentVelocity) > 0.01) {
          // Apply opposing voltage proportional to velocity to simulate braking
          motorVoltage = -Math.signum(currentVelocity) * brakeVoltage;
        } else {
          motorVoltage = 0.0;
        }
      } else {
        motorVoltage = 0.0;
      }
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
    sim.setInputVoltage(getPlantModelInput(0.25));

    Time timestamp = RobotTime.getTimestamp();
    sim.update(timestamp.minus(lastUpdateTimestamp).in(Seconds));
    lastUpdateTimestamp = timestamp;

    double simPositionRad = sim.getAngularPositionRad(); // sim in rads
    double simVelocityRadPerSec = sim.getAngularVelocityRadPerSec();

    double motorRotations = simPositionRad / (2.0 * Math.PI); // ctre expects rotations
    double motorRotPerSec = simVelocityRadPerSec / (2.0 * Math.PI);
    motorState.setRawRotorPosition(motorRotations);
    motorState.setRotorVelocity(motorRotPerSec);

    this.lastClosedLoopError = talon.getClosedLoopError().getValueAsDouble();
    this.lastCheckedMMAtTarget = talon.getMotionMagicAtTarget().getValue();

    Logger.recordOutput(pb.makePath("Sim", "Plant Input Voltage"), sim.getInputVoltage());
    Logger.recordOutput(pb.makePath("Sim", "Motor Position Rotations"), motorRotations);
    Logger.recordOutput(pb.makePath("Sim", "Motor Velocity RPS"), motorRotPerSec);
    Logger.recordOutput(
        pb.makePath("Sim", "TalonFX Reported Position"), talon.getPosition().getValueAsDouble());
    Logger.recordOutput(
        pb.makePath("Sim", "TalonFX Reported Rotor Position"),
        talon.getRotorPosition().getValueAsDouble());
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
    inputs.closedLoopError = this.lastClosedLoopError;
  }

  @Override
  public void setCurrentPositionAsZero() {
    // Call parent to set TalonFX encoder position to zero
    super.setCurrentPositionAsZero();

    // Also reset the DCMotorSim state, preserving current velocity
    double currentVelocityRadPerSec = sim.getAngularVelocityRadPerSec();
    sim.setState(0.0, currentVelocityRadPerSec);
  }

  @Override
  public void setCurrentPosition(Angle position) {
    // Call parent to set TalonFX encoder position
    super.setCurrentPosition(position);

    // Also update the DCMotorSim state so readInputs returns the correct position
    // Position comes in as mechanism rotations, convert to radians for the sim
    double mechanismRotations = position.in(Rotations);
    double motorRotations = mechanismRotations / config.unitToRotorRatio;
    double positionRad = motorRotations * 2.0 * Math.PI;

    // Preserve current velocity when setting position
    double currentVelocityRadPerSec = sim.getAngularVelocityRadPerSec();
    sim.setState(positionRad, currentVelocityRadPerSec);
  }

  @AutoLogOutput
  public double getClosedLoopError() {
    return this.lastClosedLoopError;
  }

  public boolean isMagicMotionAtTarget() {
    return this.lastCheckedMMAtTarget;
  }
}
