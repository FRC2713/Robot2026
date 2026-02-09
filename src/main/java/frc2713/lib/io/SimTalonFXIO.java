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
import frc2713.lib.util.CTREUtil;
import frc2713.lib.util.RobotTime;
import java.util.concurrent.atomic.AtomicReference;

public class SimTalonFXIO extends TalonFXIO {
  protected DCMotorSim sim;
  private Notifier simNotifier = null;
  private Time lastUpdateTimestamp = Seconds.of(0.0);

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
    // simulation.
    // The gearing parameter is the gear ratio (rotor rotations per mechanism rotation).
    // DCMotorSim outputs mechanism-side position/velocity when gearing is applied.
    this(
        config,
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), config.momentOfInertia, config.unitToRotorRatio),
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

  protected void updateSimState() {
    var motorState = talon.getSimState();
    motorState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Get motor output - use voltage if available, otherwise convert torque current to voltage
    double motorVoltage = motorState.getMotorVoltage();
    double torqueCurrent = motorState.getTorqueCurrent();

    // If torque current is being used (FOC control), convert to equivalent voltage
    // For DC motor: V = I * R + ω * kE (back-EMF)
    // DCMotorSim expects voltage input and internally computes: I = (V - ω * kE) / R
    // So to command a specific current I: V = I * R + ω * kE
    if (config.useFOC && Math.abs(torqueCurrent) > 0.1 && Math.abs(motorVoltage) < 0.1) {
      DCMotor motor = DCMotor.getKrakenX60Foc(1);
      double omegaRadPerSec = sim.getAngularVelocityRadPerSec();
      // V = I * R + ω * kE (kE = kV in radians, which equals 1/KvRadPerSecPerVolt)
      double backEmfConstant = 1.0 / motor.KvRadPerSecPerVolt;
      motorVoltage = torqueCurrent * motor.rOhms + omegaRadPerSec * backEmfConstant;
    }

    sim.setInputVoltage(motorVoltage);

    Time timestamp = RobotTime.getTimestamp();
    sim.update(timestamp.minus(lastUpdateTimestamp).in(Seconds));
    lastUpdateTimestamp = timestamp;

    // DCMotorSim with gearing outputs mechanism-side position/velocity.
    // TalonFX sim state expects rotor (motor shaft) values.
    // Convert: rotor = mechanism * unitToRotorRatio
    double mechanismPositionRad = sim.getAngularPositionRad();
    double mechanismVelocityRadPerSec = sim.getAngularVelocityRadPerSec();

    double rotorRotations = (mechanismPositionRad / (2.0 * Math.PI)) * config.unitToRotorRatio;
    double rotorRotPerSec =
        (mechanismVelocityRadPerSec / (2.0 * Math.PI)) * config.unitToRotorRatio;
    motorState.setRawRotorPosition(rotorRotations);
    motorState.setRotorVelocity(rotorRotPerSec);

    this.lastClosedLoopError = talon.getClosedLoopError().getValueAsDouble();
    this.lastCheckedMMAtTarget = talon.getMotionMagicAtTarget().getValue();
  }

  @Override
  public void readInputs(MotorInputs inputs) {
    // Note that in SIM we dont call BaseStatusSignal.refreshAll(signals) to avoid CAN errors
    // so the inputs have to come from the sim. There might still be some CAN errors at init.

    double simPositionRad = sim.getAngularPositionRad();
    double simVelocityRadPerSec = sim.getAngularVelocityRadPerSec();

    // DCMotorSim with gearing outputs mechanism-side position/velocity (not motor shaft).
    // Just convert from radians to rotations - no gear ratio conversion needed here.
    double mechanismRotations = simPositionRad / (2.0 * Math.PI);
    double mechanismRotPerSec = simVelocityRadPerSec / (2.0 * Math.PI);

    inputs.position = Rotations.of(mechanismRotations);
    inputs.velocity = RotationsPerSecond.of(mechanismRotPerSec);
    inputs.appliedVolts = Volts.of(sim.getInputVoltage());
    inputs.currentStatorAmps = Amps.of(sim.getCurrentDrawAmps());
    inputs.currentSupplyAmps = Amps.of(sim.getCurrentDrawAmps());
    inputs.currenTorqueAmps = Amps.of(sim.getCurrentDrawAmps());
    // rawRotorPosition is motor shaft position: mechanism * unitToRotorRatio
    inputs.rawRotorPosition = Rotations.of(mechanismRotations * config.unitToRotorRatio);
    inputs.closedLoopError = this.lastClosedLoopError;
    inputs.isMotionMagicAtTarget = this.lastCheckedMMAtTarget;

    // Check for tunable gains updates (same as TalonFXIO)
    if (this.config.tunable && tunableGains != null) {
      tunableGains.ifChanged(
          this.hashCode(),
          (gains, motionMagic) -> {
            this.config.fxConfig.Slot0 = gains;
            this.config.fxConfig.MotionMagic = motionMagic;
            CTREUtil.applyConfiguration(talon, this.config.fxConfig);
          });
    }
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
    // Position comes in as mechanism rotations, convert to mechanism radians for the sim.
    // DCMotorSim with gearing expects mechanism-side position.
    double mechanismRotations = position.in(Rotations);
    double positionRad = mechanismRotations * 2.0 * Math.PI;

    // Preserve current velocity when setting position
    double currentVelocityRadPerSec = sim.getAngularVelocityRadPerSec();
    sim.setState(positionRad, currentVelocityRadPerSec);
  }
}
