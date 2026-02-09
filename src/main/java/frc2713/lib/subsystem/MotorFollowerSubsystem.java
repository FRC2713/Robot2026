package frc2713.lib.subsystem;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc2713.lib.io.AdvantageScopePathBuilder;
import frc2713.lib.io.MotorIO;
import frc2713.lib.io.MotorInputsAutoLogged;
import frc2713.lib.util.RobotTime;
import frc2713.lib.util.Util;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * A subsystem that manages two motors, where one follows the other. The left motor is the leader
 * and the right motor is the follower.
 */
public class MotorFollowerSubsystem<MI extends MotorInputsAutoLogged, IO extends MotorIO>
    extends SubsystemBase {
  protected final IO leftIO;
  protected final IO rightIO;
  protected final MI leftInputs;
  protected final MI rightInputs;
  protected final TalonFXSubsystemConfig leftConfig;
  protected final TalonFXSubsystemConfig rightConfig;
  protected final AdvantageScopePathBuilder pb;

  protected Angle positionSetpoint = Radians.of(0.0);
  protected AngularVelocity velocitySetpoint = RotationsPerSecond.of(0.0);

  public MotorFollowerSubsystem(
      String name,
      TalonFXSubsystemConfig leftConfig,
      TalonFXSubsystemConfig rightConfig,
      MI leftInputs,
      MI rightInputs,
      IO leftIO,
      IO rightIO) {
    // Set the subsystem name
    super(name);
    this.leftConfig = leftConfig;
    this.rightConfig = rightConfig;
    this.leftInputs = leftInputs;
    this.rightInputs = rightInputs;
    this.leftIO = leftIO;
    this.rightIO = rightIO;

    this.pb = new AdvantageScopePathBuilder(this.getName());

    // Set the default command to stop both motors
    setDefaultCommand(
        this.dutyCycleCommand(() -> 0.0)
            .withName(pb.makeName("DefaultCommand"))
            .ignoringDisable(true));

    this.rightIO.follow(leftConfig.talonCANID, true);
  }

  @Override
  public void periodic() {
    Time timestamp = RobotTime.getTimestamp();
    leftIO.readInputs(leftInputs);
    rightIO.readInputs(rightInputs);
    Logger.processInputs(getName() + "/Left", leftInputs);
    Logger.processInputs(getName() + "/Right", rightInputs);
    Logger.recordOutput(pb.makePath("LatencyPeriodSec"), RobotTime.getTimestamp().minus(timestamp));
    Logger.recordOutput(
        pb.makePath("currentCommand"),
        (getCurrentCommand() == null) ? "Default" : getCurrentCommand().getName());
  }

  /**
   * Convert subsystem position to the position value sent to the motor controller. Since TalonFX is
   * configured with SensorToMechanismRatio, control requests use mechanism units directly - the
   * controller handles the conversion internally.
   *
   * @param subsystemPosition The desired mechanism position
   * @return The position to send to the motor controller (same as input)
   */
  protected Angle convertSubsystemPositionToMotorPosition(Angle subsystemPosition) {
    return subsystemPosition;
  }

  /**
   * Convert a linear distance to mechanism rotations for the motor controller.
   *
   * @param subsystemPosition Desired distance
   * @return the number of mechanism rotations to achieve that distance
   */
  protected Angle convertSubsystemPositionToMotorPosition(Distance subsystemPosition) {
    return Rotations.of(subsystemPosition.in(Meters) * leftConfig.unitRotationsPerMeter);
  }

  // IO Implementations

  /**
   * Sets the Motion Magic configuration for both motors.
   *
   * @param config The Motion Magic configuration to set.
   */
  protected void setMotionMagicConfigImpl(MotionMagicConfigs config) {
    leftIO.setMotionMagicConfig(config);
  }

  /**
   * Sets the neutral mode of both motors (e.g., brake or coast).
   *
   * @param mode The desired neutral mode.
   */
  protected void setNeutralModeImpl(NeutralModeValue mode) {
    Logger.recordOutput(pb.makePath("API", "setNeutralModeImpl", "mode"), mode);
    leftIO.setNeutralMode(mode);
  }

  /**
   * Sets both motors to the specified duty cycle in open loop control.
   *
   * @param dutyCycle The desired duty cycle (-1.0 to 1.0).
   */
  protected void setOpenLoopDutyCycleImpl(double dutyCycle) {
    Logger.recordOutput(pb.makePath("API", "setOpenLoopDutyCycleImpl", "dutyCycle"), dutyCycle);
    leftIO.setOpenLoopDutyCycle(dutyCycle);
  }

  /**
   * Sets both motors to the specified voltage.
   *
   * @param voltage The desired voltage.
   */
  protected void setVoltageImpl(Voltage voltage) {
    Logger.recordOutput(pb.makePath("API", "setVoltageImpl", "voltage"), voltage);
    Logger.recordOutput(pb.makePath("API", "setVoltageImpl", "units"), voltage.unit().toString());
    leftIO.setVoltageOutput(voltage);
  }

  /**
   * Sets both motors to the specified position setpoint. This is converted to rotations by the
   * TalonFX API.
   *
   * @param position The desired position setpoint.
   */
  protected void setPositionSetpointImpl(Angle position) {
    positionSetpoint = position;
    Logger.recordOutput(pb.makePath("API", "setPositionSetpointImpl", "position"), position);
    Logger.recordOutput(
        pb.makePath("API", "setPositionSetpointImpl", "units"), position.unit().toString());
    leftIO.setPositionSetpoint(position);
  }

  /**
   * Sets both motors to the specified position setpoint using Motion Magic control. This is
   * converted to rotations by the TalonFX API.
   *
   * @param position The desired position setpoint.
   * @param slot The PID slot to use for the Motion Magic control.
   */
  protected void setMotionMagicSetpointImpl(Angle position, int slot) {
    positionSetpoint = position;
    Logger.recordOutput(pb.makePath("API", "setMotionMagicSetpointImpl", "position"), position);
    Logger.recordOutput(
        pb.makePath("API", "setMotionMagicSetpointImpl", "units"), position.unit().toString());
    Logger.recordOutput(pb.makePath("API", "setMotionMagicSetpointImpl", "slot"), slot);
    leftIO.setMotionMagicSetpoint(position, slot);
  }

  /**
   * Sets both motors to the specified position setpoint using Motion Magic control with dynamic
   * parameters. This is converted to rotations by the TalonFX API.
   *
   * @param position The desired position setpoint.
   * @param config The Motion Magic configuration to use.
   * @param slot The PID slot to use for the Motion Magic control.
   */
  protected void setMotionMagicSetpointImpl(Angle position, MotionMagicConfigs config, int slot) {
    positionSetpoint = position;

    AngularVelocity velocity = config.getMotionMagicCruiseVelocityMeasure();
    AngularAcceleration acceleration = config.getMotionMagicAccelerationMeasure();
    Velocity<AngularAccelerationUnit> jerk = config.getMotionMagicJerkMeasure();
    Logger.recordOutput(
        pb.makePath("API", "setMotionMagicSetpointImpDynamic", "Position"), position);
    Logger.recordOutput(
        pb.makePath("API", "setMotionMagicSetpointImpDynamic", "Units"),
        position.unit().toString());
    Logger.recordOutput(
        pb.makePath("API", "setMotionMagicSetpointImpDynamic", "Velocity"), velocity);
    Logger.recordOutput(
        pb.makePath("API", "setMotionMagicSetpointImpDynamic", "Accel"), acceleration);
    Logger.recordOutput(pb.makePath("API", "setMotionMagicSetpointImpDynamic", "Jerk"), jerk);
    Logger.recordOutput(pb.makePath("API", "setMotionMagicSetpointImpDynamic", "Slot"), slot);
    leftIO.setMotionMagicSetpoint(position, velocity, acceleration, jerk, slot);
  }

  /**
   * Sets both motors to the specified position setpoint using Motion Magic control with dynamic
   * parameters and feedforward. This is converted to rotations by the TalonFX API.
   *
   * @param position The desired position setpoint.
   * @param config The Motion Magic configuration to use.
   * @param feedfoward The feedforward value to apply.
   * @param slot The PID slot to use for the Motion Magic control.
   */
  protected void setMotionMagicSetpointImpl(
      Angle position, MotionMagicConfigs config, double feedfoward, int slot) {
    positionSetpoint = position;

    AngularVelocity velocity = config.getMotionMagicCruiseVelocityMeasure();
    AngularAcceleration acceleration = config.getMotionMagicAccelerationMeasure();
    Velocity<AngularAccelerationUnit> jerk = config.getMotionMagicJerkMeasure();
    Logger.recordOutput(
        pb.makePath("API", "setMotionMagicSetpointImpDynamic", "Position"), position);
    Logger.recordOutput(
        pb.makePath("API", "setMotionMagicSetpointImpDynamic", "Units"),
        position.unit().toString());
    Logger.recordOutput(
        pb.makePath("API", "setMotionMagicSetpointImpDynamic", "Velocity"), velocity);
    Logger.recordOutput(
        pb.makePath("API", "setMotionMagicSetpointImpDynamic", "Accel"), acceleration);
    Logger.recordOutput(pb.makePath("API", "setMotionMagicSetpointImpDynamic", "Jerk"), jerk);
    Logger.recordOutput(pb.makePath("API", "setMotionMagicSetpointImpDynamic", "Slot"), slot);
    Logger.recordOutput(
        pb.makePath("API", "setMotionMagicSetpointImpDynamic", "Feedforward"), feedfoward);
    leftIO.setMotionMagicSetpoint(position, velocity, acceleration, jerk, slot, feedfoward);
  }

  /**
   * Sets both motors to the specified velocity setpoint. This is converted to rotations per second
   * by the TalonFX API.
   *
   * @param setpoint The desired velocity setpoint.
   * @param slot The PID slot to use for the velocity control.
   */
  protected void setVelocitySetpointImpl(AngularVelocity setpoint, int slot) {
    velocitySetpoint = setpoint;
    Logger.recordOutput(pb.makePath("API", "setVelocitySetpointImpl", "Velocity"), setpoint);
    Logger.recordOutput(
        pb.makePath("API", "setVelocitySetpointImpl", "Units"), setpoint.unit().toString());
    Logger.recordOutput(pb.makePath("API", "setVelocitySetpointImpl", "Slot"), slot);
    leftIO.setVelocitySetpoint(setpoint, slot);
  }

  // Getters
  /**
   * Gets the current position of the left motor.
   *
   * @return The current position of the left motor.
   */
  public Angle getLeftCurrentPosition() {
    return leftInputs.position;
  }

  /**
   * Gets the current position of the right motor.
   *
   * @return The current position of the right motor.
   */
  public Angle getRightCurrentPosition() {
    return rightInputs.position;
  }

  /**
   * Gets the current velocity of the left motor.
   *
   * @return The current velocity of the left motor.
   */
  public AngularVelocity getLeftCurrentVelocity() {
    return leftInputs.velocity;
  }

  /**
   * Gets the current velocity of the right motor.
   *
   * @return The current velocity of the right motor.
   */
  public AngularVelocity getRightCurrentVelocity() {
    return rightInputs.velocity;
  }

  /**
   * Gets the current position setpoint of the motors.
   *
   * @return The current position setpoint of the motors.
   */
  public Angle getPositionSetpoint() {
    return positionSetpoint;
  }

  /**
   * Sets both motors to the specified torque current in FOC control.
   *
   * @param current The desired torque current.
   */
  public void setTorqueCurrentFOCImpl(Current current) {
    Logger.recordOutput(pb.makePath("API", "setTorqueCurrentFoC", "Current"), current);
    Logger.recordOutput(
        pb.makePath("API", "setTorqueCurrentFoC", "Units"), current.unit().toString());

    leftIO.setTorqueCurrentFOC(current);
  }

  /** Sets the current position of both motors as zero. */
  protected void setCurrentPositionAsZero() {
    leftIO.setCurrentPositionAsZero();
  }

  /**
   * Sets the current position of both motors.
   *
   * @param position The desired current position.
   */
  public void setCurrentPosition(Angle position) {
    leftIO.setCurrentPosition(position);
  }

  // Command Generators

  /**
   * Creates a command that sets the Motion Magic configuration for both motors.
   *
   * @param configs The Motion Magic configuration to set.
   * @return A command that sets the Motion Magic configuration.
   */
  public Command setMotionMagicConfigCommand(MotionMagicConfigs configs) {
    // Not taking requirements is intentional here.
    return new InstantCommand(() -> setMotionMagicConfigImpl(configs));
  }

  /**
   * Creates a command that sets both motors to the specified duty cycle in open loop control.
   *
   * @param dutyCycle The desired duty cycle supplier (-1.0 to 1.0).
   * @return A command that sets both motors to the specified duty cycle. When the command ends, the
   *     motors are set to 0.0 duty cycle.
   */
  public Command dutyCycleCommand(DoubleSupplier dutyCycle) {
    return runEnd(
            () -> {
              setOpenLoopDutyCycleImpl(dutyCycle.getAsDouble());
            },
            () -> {
              setOpenLoopDutyCycleImpl(0.0);
            })
        .withName(pb.makeName("DutyCycleControl"));
  }

  /**
   * Creates a command that sets both motors to the specified duty cycle in open loop control.
   *
   * @param dutyCycle The desired duty cycle supplier (-1.0 to 1.0).
   * @return A command that sets both motors to the specified duty cycle. When the command ends, the
   *     motors are not modified.
   */
  public Command dutyCycleCommandNoEnd(DoubleSupplier dutyCycle) {
    return runEnd(
            () -> {
              setOpenLoopDutyCycleImpl(dutyCycle.getAsDouble());
            },
            () -> {})
        .withName(pb.makeName("DutyCycleControl"));
  }

  /**
   * Creates a command that sets both motors to the specified voltage.
   *
   * @param voltage The desired voltage supplier.
   * @return A command that sets both motors to the specified voltage. When the command ends, the
   *     motors are set to 0.0 volts.
   */
  public Command voltageCommand(Supplier<Voltage> voltage) {
    return runEnd(
            () -> {
              setVoltageImpl(voltage.get());
            },
            () -> {
              setVoltageImpl(Volts.of(0.0));
            })
        .withName(pb.makeName("VoltageControl"));
  }

  /**
   * Creates a command that sets both motors to the specified velocity setpoint.
   *
   * @param velocitySupplier The desired velocity setpoint supplier.
   * @return A command that sets both motors to the specified velocity setpoint.
   */
  public Command velocitySetpointCommand(Supplier<AngularVelocity> velocitySupplier) {
    return velocitySetpointCommand(velocitySupplier, 0);
  }

  /**
   * Creates a command that sets both motors to the specified velocity setpoint.
   *
   * @param velocitySupplier The desired velocity setpoint supplier.
   * @param slot The PID slot to use for the velocity control.
   * @return A command that sets both motors to the specified velocity setpoint.
   */
  public Command velocitySetpointCommand(Supplier<AngularVelocity> velocitySupplier, int slot) {
    return runEnd(
            () -> {
              setVelocitySetpointImpl(velocitySupplier.get(), slot);
            },
            () -> {})
        .withName(pb.makeName("VelocityControl"));
  }

  /**
   * Creates a command that sets both motors to brake mode when started and coast mode when ended.
   *
   * @return A command that sets both motors to brake mode when started and coast mode when ended.
   */
  public Command setCoast() {
    return startEnd(
            () -> setNeutralModeImpl(NeutralModeValue.Coast),
            () -> setNeutralModeImpl(NeutralModeValue.Brake))
        .withName(pb.makeName("CoastMode"))
        .ignoringDisable(true);
  }

  /**
   * Creates a command that sets both motors to the specified position setpoint.
   *
   * @param positionSupplier The desired position setpoint supplier.
   * @return A command that sets both motors to the specified position setpoint.
   */
  public Command positionSetpointCommand(Supplier<Angle> positionSupplier) {
    return runEnd(
            () -> {
              setPositionSetpointImpl(positionSupplier.get());
            },
            () -> {})
        .withName(pb.makeName("PositionSetpointCommand"));
  }

  /**
   * Creates a command that sets both motors to the specified position setpoint and ends when both
   * positions are within the specified epsilon of the target.
   *
   * @param positionSupplier The desired position setpoint supplier.
   * @param epsilonSupplier The acceptable error margin supplier.
   * @return A command that sets both motors to the specified position setpoint and ends when on
   *     target.
   */
  public Command positionSetpointUntilOnTargetCommand(
      Supplier<Angle> positionSupplier, Supplier<Angle> epsilonSupplier) {
    return new ParallelDeadlineGroup(
            new WaitUntilCommand(
                () ->
                    Util.epsilonEquals(
                            positionSupplier.get(), leftInputs.position, epsilonSupplier.get())
                        && Util.epsilonEquals(
                            positionSupplier.get(), rightInputs.position, epsilonSupplier.get())),
            positionSetpointCommand(positionSupplier))
        .withName(pb.makeName("PositionUntilOnTargetControl"));
  }

  /**
   * Creates a command that sets both motors to the specified velocity setpoint and ends when both
   * velocities are within the specified epsilon of the target.
   *
   * @param velocitySupplier The desired velocity setpoint supplier.
   * @param epsilonSupplier The acceptable error margin supplier.
   * @return A command that sets both motors to the specified velocity setpoint and ends when on
   *     target.
   */
  public Command velocitySetpointUntilOnTargetCommand(
      Supplier<AngularVelocity> velocitySupplier, Supplier<AngularVelocity> epsilonSupplier) {
    return new ParallelDeadlineGroup(
            new WaitUntilCommand(
                () ->
                    Util.epsilonEquals(
                            velocitySupplier.get(), leftInputs.velocity, epsilonSupplier.get())
                        && Util.epsilonEquals(
                            velocitySupplier.get(), rightInputs.velocity, epsilonSupplier.get())),
            velocitySetpointCommand(velocitySupplier))
        .withName(pb.makeName("VelocityUntilOnTargetControl"));
  }

  /**
   * Creates a command that sets both motors to the specified position setpoint using Motion Magic
   * control.
   *
   * @param positionSupplier The desired position setpoint supplier.
   * @param slot The PID slot to use for the Motion Magic control.
   * @return A command that sets both motors to the specified position setpoint using Motion Magic
   *     control.
   */
  public Command motionMagicSetpointCommand(Supplier<Angle> positionSupplier, int slot) {
    return runEnd(
            () -> {
              setMotionMagicSetpointImpl(positionSupplier.get(), slot);
            },
            () -> {})
        .withName(pb.makeName("motionMagicSetpointCommand"));
  }

  /**
   * Creates a command that sets both motors to the specified position setpoint using Motion Magic
   * control with dynamic parameters.
   *
   * @param positionSupplier The desired position setpoint supplier.
   * @param configSupplier The Motion Magic configuration supplier.
   * @param slot The PID slot to use for the Motion Magic control.
   * @return A command that sets both motors to the specified position setpoint using Motion Magic
   *     control with dynamic parameters.
   */
  public Command motionMagicSetpointCommand(
      Supplier<Angle> positionSupplier, Supplier<MotionMagicConfigs> configSupplier, int slot) {
    return runEnd(
            () -> {
              setMotionMagicSetpointImpl(positionSupplier.get(), configSupplier.get(), slot);
            },
            () -> {})
        .withName(pb.makeName("dynamicMotionMagicSetpointCommand"));
  }

  /**
   * Creates a command that sets both motors to the specified position setpoint using Motion Magic
   * control with dynamic parameters and feedforward.
   *
   * @param unitSupplier The desired position setpoint supplier.
   * @param configSupplier The Motion Magic configuration supplier.
   * @param feedforward The feedforward supplier.
   * @param slot The PID slot to use for the Motion Magic control.
   * @return A command that sets both motors to the specified position setpoint using Motion Magic
   *     control with dynamic parameters and feedforward.
   */
  public Command motionMagicSetpointCommand(
      Supplier<Angle> unitSupplier,
      Supplier<MotionMagicConfigs> configSupplier,
      Supplier<Double> feedforward,
      int slot) {
    return runEnd(
            () -> {
              setMotionMagicSetpointImpl(
                  unitSupplier.get(), configSupplier.get(), feedforward.get(), slot);
            },
            () -> {})
        .withName(pb.makeName("dynamicMotionMagicSetpointCommand"));
  }

  /**
   * Creates a command that sets both motors to the specified position setpoint using Motion Magic
   * control and ends when both positions are within the specified tolerance of the target.
   *
   * @param setpointSupplier The desired position setpoint supplier.
   * @param tolerance The acceptable error margin.
   * @param slot The PID slot to use for the Motion Magic control.
   * @return A command that sets both motors to the specified position setpoint using Motion Magic
   *     control and ends when on target.
   */
  public Command motionMagicSetpointCommandBlocking(
      Supplier<Angle> setpointSupplier, Angle tolerance, int slot) {
    return motionMagicSetpointCommand(setpointSupplier, slot)
        .until(
            () ->
                Util.epsilonEquals(getLeftCurrentPosition(), setpointSupplier.get(), tolerance)
                    && Util.epsilonEquals(
                        getRightCurrentPosition(), setpointSupplier.get(), tolerance));
  }

  /**
   * Creates a command that sets both motors to the specified position setpoint using Motion Magic
   * control with dynamic parameters and ends when both positions are within the specified tolerance
   * of the target.
   *
   * @param setpointSupplier The desired position setpoint supplier.
   * @param configSupplier The Motion Magic configuration supplier.
   * @param tolerance The acceptable error margin.
   * @param slot The PID slot to use for the Motion Magic control.
   * @return A command that sets both motors to the specified position setpoint using Motion Magic
   *     control with dynamic parameters and ends when on target.
   */
  public Command motionMagicSetpointCommandBlocking(
      Supplier<Angle> setpointSupplier,
      Supplier<MotionMagicConfigs> configSupplier,
      Angle tolerance,
      int slot) {
    return motionMagicSetpointCommand(setpointSupplier, configSupplier, slot)
        .until(
            () ->
                Util.epsilonEquals(getLeftCurrentPosition(), setpointSupplier.get(), tolerance)
                    && Util.epsilonEquals(
                        getRightCurrentPosition(), setpointSupplier.get(), tolerance));
  }

  /**
   * Creates a command that sets both motors to the specified position setpoint using Motion Magic
   * control with dynamic parameters and feedforward, and ends when both positions are within the
   * specified tolerance of the target.
   *
   * @param setpointSupplier The desired position setpoint supplier.
   * @param configSupplier The Motion Magic configuration supplier.
   * @param feedforward The feedforward supplier.
   * @param tolerance The acceptable error margin.
   * @param slot The PID slot to use for the Motion Magic control.
   * @return A command that sets both motors to the specified position setpoint using Motion Magic
   *     control with dynamic parameters and feedforward, and ends when on target.
   */
  public Command motionMagicSetpointCommandBlocking(
      Supplier<Angle> setpointSupplier,
      Supplier<MotionMagicConfigs> configSupplier,
      Supplier<Double> feedforward,
      Angle tolerance,
      int slot) {
    return motionMagicSetpointCommand(setpointSupplier, configSupplier, feedforward, slot)
        .until(
            () ->
                Util.epsilonEquals(getLeftCurrentPosition(), setpointSupplier.get(), tolerance)
                    && Util.epsilonEquals(
                        getRightCurrentPosition(), setpointSupplier.get(), tolerance));
  }

  // Overloads for convenience
  public Command motionMagicSetpointCommand(Supplier<Angle> setpointSupplier) {
    return motionMagicSetpointCommand(setpointSupplier, 0);
  }

  public Command motionMagicSetpointCommand(
      Supplier<Angle> setpointSupplier, Supplier<MotionMagicConfigs> configSupplier) {
    return motionMagicSetpointCommand(setpointSupplier, configSupplier, 0);
  }

  public Command motionMagicSetpointCommandBlocking(
      Supplier<Angle> setpointSupplier, Angle tolerance) {
    return motionMagicSetpointCommandBlocking(setpointSupplier, tolerance, 0);
  }

  public Command motionMagicSetpointCommandBlocking(
      Supplier<Angle> setpointSupplier,
      Supplier<MotionMagicConfigs> configSupplier,
      Angle tolerance) {
    return motionMagicSetpointCommandBlocking(setpointSupplier, configSupplier, tolerance, 0);
  }

  /**
   * Creates a command that sets both motors to the specified torque current in FOC control.
   *
   * @param current The desired torque current supplier.
   * @return A command that sets both motors to the specified torque current in FOC control.
   */
  public Command setTorqueCurrentFOC(Supplier<Current> current) {
    return runEnd(
            () -> {
              setTorqueCurrentFOCImpl(current.get());
            },
            () -> {})
        .withName(pb.makeName("torqueCurrentFOCCommand"));
  }

  /**
   * Creates a command that temporarily disables software limits while running.
   *
   * @return A command that temporarily disables software limits while running.
   */
  protected Command withoutLimitsTemporarily() {
    var prevLeft =
        new Object() {
          boolean fwd = false;
          boolean rev = false;
        };

    return Commands.startEnd(
        () -> {
          prevLeft.fwd = leftConfig.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable;
          prevLeft.rev = leftConfig.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable;

          leftIO.setEnableSoftLimits(false, false);
        },
        () -> {
          leftIO.setEnableSoftLimits(prevLeft.fwd, prevLeft.rev);
        });
  }
}
