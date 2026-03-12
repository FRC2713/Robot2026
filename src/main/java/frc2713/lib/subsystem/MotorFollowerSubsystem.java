package frc2713.lib.subsystem;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc2713.lib.io.MotorIO;
import frc2713.lib.io.MotorInputsAutoLogged;
import frc2713.lib.subsystem.TalonFXSubsystemConfig.GeneralControlMode;
import frc2713.lib.util.RobotTime;
import frc2713.lib.util.Util;
import org.littletonrobotics.junction.Logger;

/**
 * A subsystem that manages two motors, where one follows the other. The follower motor
 * automatically follows the leader's movements.
 */
public class MotorFollowerSubsystem<MI extends MotorInputsAutoLogged, IO extends MotorIO>
    extends MotorSubsystem<MI, IO> {
  protected final IO followerIO;
  protected final MI followerInputs;
  protected final TalonFXSubsystemConfig followerConfig;

  public MotorFollowerSubsystem(
      String name,
      TalonFXSubsystemConfig leaderConfig,
      TalonFXSubsystemConfig followerConfig,
      MI leaderInputs,
      MI followerInputs,
      IO leaderIO,
      IO followerIO) {
    // Call the parent constructor with the leader motor configuration
    super(leaderConfig, leaderInputs, leaderIO);
    setName(name);
    this.followerConfig = followerConfig;
    this.followerInputs = followerInputs;
    this.followerIO = followerIO;

    // Set the default command to stop both motors
    // setDefaultCommand(
    //     this.dutyCycleCommand(() -> 0.0)
    //         .withName(pb.makeName("DefaultCommand"))
    //         .ignoringDisable(true));

    this.followerIO.follow(config.talonCANID, true);
  }

  @Override
  public void periodic() {
    // Call the parent periodic for the leader motor
    super.periodic();

    // Handle the follower motor separately
    Time timestamp = RobotTime.getTimestamp();
    followerIO.readInputs(followerInputs);
    Logger.processInputs(getName() + "/Follower", followerInputs);
    Logger.recordOutput(pb.makePath("LatencyPeriodSec"), RobotTime.getTimestamp().minus(timestamp));
    Logger.recordOutput(
        pb.makePath("currentCommand"),
        (getCurrentCommand() == null) ? "Default" : getCurrentCommand().getName());

    // Log setpoints for comparison with measurements (in same units as inputs)
    Logger.recordOutput(
        pb.makePath("Setpoints", "velocityRotPerSec"), velocitySetpoint.in(RotationsPerSecond));
    Logger.recordOutput(pb.makePath("Setpoints", "positionRot"), positionSetpoint.in(Rotations));
    // Also log measurements as doubles for easy comparison
    Logger.recordOutput(
        pb.makePath("Measurements", "leaderVelocityRotPerSec"),
        inputs.velocity.in(RotationsPerSecond));
    Logger.recordOutput(
        pb.makePath("Measurements", "followerVelocityRotPerSec"),
        followerInputs.velocity.in(RotationsPerSecond));
    Logger.recordOutput(
        pb.makePath("Measurements", "leaderAppliedVolts"), inputs.appliedVolts.in(Volts));
    Logger.recordOutput(
        pb.makePath("Measurements", "followerAppliedVolts"), followerInputs.appliedVolts.in(Volts));
    Logger.recordOutput(pb.makePath("Measurements", "closedLoopError"), inputs.closedLoopError);
  }

  // Getters
  /**
   * Gets the current position of the leader motor.
   *
   * @return The current position of the leader motor.
   */
  public Angle getLeaderCurrentPosition() {
    return inputs.position;
  }

  /**
   * Gets the current position of the follower motor.
   *
   * @return The current position of the follower motor.
   */
  public Angle getFollowerCurrentPosition() {
    return followerInputs.position;
  }

  /**
   * Gets the current velocity of the leader motor.
   *
   * @return The current velocity of the leader motor.
   */
  public AngularVelocity getLeaderCurrentVelocity() {
    return inputs.velocity;
  }

  /**
   * Gets the current velocity of the follower motor.
   *
   * @return The current velocity of the follower motor.
   */
  public AngularVelocity getFollowerCurrentVelocity() {
    return followerInputs.velocity;
  }

  // Command Generators - Most commands are inherited from MotorSubsystem
  // Only follower-specific commands that need to check both motors are defined here

  /**
   * Creates a command that temporarily disables software limits while running.
   *
   * @return A command that temporarily disables software limits while running.
   */
  protected Command withoutLimitsTemporarily() {
    var prevLeader =
        new Object() {
          boolean fwd = false;
          boolean rev = false;
        };

    return Commands.startEnd(
        () -> {
          prevLeader.fwd = config.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable;
          prevLeader.rev = config.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable;

          io.setEnableSoftLimits(false, false);
        },
        () -> {
          io.setEnableSoftLimits(prevLeader.fwd, prevLeader.rev);
        });
  }

  @Override
  public boolean atTarget() {
    var atTarget = false;
    if (config.generalControlMode == GeneralControlMode.POSITION) {
      atTarget =
          Util.epsilonEquals(
              getCurrentPosition(), positionSetpoint, config.acceptablePositionError);
    } else if (config.generalControlMode == GeneralControlMode.VELOCITY) {
      atTarget =
          Util.epsilonEquals(
              getCurrentVelocity(), velocitySetpoint, config.acceptableVelocityError);
    }
    Logger.recordOutput(pb.makePath("AtTarget"), atTarget);
    return atTarget;
  }
}
