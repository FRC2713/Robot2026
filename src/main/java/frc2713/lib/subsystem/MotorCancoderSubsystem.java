package frc2713.lib.subsystem;

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
}
