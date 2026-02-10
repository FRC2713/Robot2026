package frc2713.lib.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
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
import frc2713.lib.util.CTREUtil;
import frc2713.lib.util.RobotTime;

public class SimTalonFXIO extends TalonFXIO {
  protected DCMotorSim sim;
  private Notifier simNotifier = null;
  private Time lastUpdateTimestamp = null; // Initialized on first update to avoid huge dt

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
    // So to command a specific current I: V = I * R + ω * kE
    if (config.useFOC && Math.abs(torqueCurrent) > 0.1 && Math.abs(motorVoltage) < 0.1) {
      double backEmfConstant = 1.0 / (KRAKEN_X60_KV_RPS_PER_VOLT * 2.0 * Math.PI); // kE
      double rOhms = 0.025; // internal resistance from Kraken x60 specs
      motorVoltage = torqueCurrent * rOhms + sim.getAngularVelocityRadPerSec() * backEmfConstant;
    }

    sim.setInputVoltage(motorVoltage);

    Time timestamp = RobotTime.getTimestamp();
    // Initialize lastUpdateTimestamp on first update to avoid huge dt
    if (lastUpdateTimestamp == null) {
      lastUpdateTimestamp = timestamp;
    }
    double dt = timestamp.minus(lastUpdateTimestamp).in(Seconds);
    // Clamp dt to reasonable values to prevent simulation instability
    dt = Math.min(dt, 0.1);
    sim.update(dt);
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

    // Guard against NaN from uninitialized sim state
    if (Double.isNaN(simPositionRad)) simPositionRad = 0.0;
    if (Double.isNaN(simVelocityRadPerSec)) simVelocityRadPerSec = 0.0;

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

  @Override
  public void setVelocitySetpoint(AngularVelocity unitsPerSecond, int slot) {
    super.setVelocitySetpoint(unitsPerSecond, slot);
    if (Math.abs(unitsPerSecond.in(DegreesPerSecond)) < 0.1
        && config.fxConfig.MotorOutput.NeutralMode == NeutralModeValue.Brake) {}
  }
}
