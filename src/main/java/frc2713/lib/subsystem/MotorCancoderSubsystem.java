package frc2713.lib.subsystem;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc2713.lib.io.CanCoderIO;
import frc2713.lib.io.CanCoderInputsAutoLogged;
import frc2713.lib.io.MotorIO;
import frc2713.lib.io.MotorInputs;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * A MotorSubsystem that requires CANcoder inputs and IO. Use this for subsystems that have a
 * CANcoder (e.g. Turret). Subsystems without a CANcoder should extend MotorSubsystem directly.
 */
public class MotorCancoderSubsystem<MI extends MotorInputs & LoggableInputs, IO extends MotorIO>
    extends MotorSubsystem<MI, IO> {

  protected final CanCoderInputsAutoLogged cancoderInputs;
  protected final CanCoderIO cancoderIO;

  protected MotorCancoderSubsystem(
      TalonFXSubsystemConfig config,
      MI inputs,
      IO io,
      CanCoderInputsAutoLogged cancoderInputs,
      CanCoderIO cancoderIO) {
    super(config, inputs, io);
    this.cancoderInputs = cancoderInputs;
    this.cancoderIO = cancoderIO;
  }

  @Override
  public void periodic() {
    super.periodic();

    cancoderIO.readInputs(cancoderInputs);
    Logger.processInputs(getName() + "/cancoder", cancoderInputs);
  }

  /** Returns whether this subsystem has a CANcoder. Always true for MotorCancoderSubsystem. */
  public boolean hasCanCoder() {
    return true;
  }

  /**
   * Gets the CANcoder absolute position in rotations.
   *
   * @return CANcoder position in rotations, or 0 if absolutePosition is null
   */
  public Angle getCanCoderPosition() {
    return cancoderInputs.absolutePosition == null
        ? Rotations.of(0.0)
        : cancoderInputs.absolutePosition;
  }

  /**
   * Gets the CANcoder velocity in rotations per second.
   *
   * @return CANcoder velocity in rotations per second
   */
  public AngularVelocity getCanCoderVelocity() {
    return cancoderInputs.velocity;
  }
}
