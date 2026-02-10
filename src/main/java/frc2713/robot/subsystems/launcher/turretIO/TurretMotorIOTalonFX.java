package frc2713.robot.subsystems.launcher.turretIO;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.measure.Angle;
import frc2713.lib.io.TalonFXIO;
import frc2713.lib.util.CTREUtil;
import frc2713.robot.subsystems.launcher.Turret;

/**
 * TalonFX IO implementation for the turret with dual encoder support. Uses the TalonFX integrated
 * encoder as encoder 1 and an external CANCoder as encoder 2.
 */
public class TurretMotorIOTalonFX extends TalonFXIO implements TurretMotorIO {

  private final CANcoder canCoder;

  // CANCoder status signals
  private final StatusSignal<Angle> canCoderPositionSignal;

  private final BaseStatusSignal[] turretSignals;

  public TurretMotorIOTalonFX(TurretSubsystemConfig config) {
    super(config);

    // Initialize CANCoder
    this.canCoder =
        new CANcoder(
            config.canCoderCANID.getDeviceNumber(), new CANBus(config.canCoderCANID.getBus()));

    // Get position signal from CANCoder
    canCoderPositionSignal = canCoder.getAbsolutePosition();

    // Create array of CANCoder signals for batch refresh
    turretSignals = new BaseStatusSignal[] {canCoderPositionSignal};

    // Set update frequency for CANCoder signals
    CTREUtil.tryUntilOK(
        () -> BaseStatusSignal.setUpdateFrequencyForAll(50.0, turretSignals),
        canCoder.getDeviceID());
    CTREUtil.tryUntilOK(() -> canCoder.optimizeBusUtilization(), canCoder.getDeviceID());
  }

  @Override
  public void readInputs(TurretInputs inputs) {
    // Call parent to read standard motor inputs
    super.readInputs(inputs);

    // Refresh CANCoder signals
    BaseStatusSignal.refreshAll(turretSignals);

    // Read encoder 1 (TalonFX integrated encoder) - convert rotations to degrees
    double encoder1Degrees = inputs.rawRotorPosition.in(Degrees);
    inputs.encoder1PositionDegrees = encoder1Degrees;

    // Read encoder 2 (external CANCoder) - convert to degrees
    double encoder2Degrees = canCoderPositionSignal.getValue().in(Degrees);
    inputs.encoder2PositionDegrees = encoder2Degrees;

    // Compute turret position from both encoders using the Vernier algorithm
    double computedPosition = Turret.turretPositionFromEncoders(encoder1Degrees, encoder2Degrees);
    inputs.computedTurretPositionDegrees = computedPosition;
  }
}
