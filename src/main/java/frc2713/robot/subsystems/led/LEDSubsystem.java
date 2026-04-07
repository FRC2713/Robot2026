package frc2713.robot.subsystems.led;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/** LED subsystem for controlling CANdle-based LEDs. */
public class LEDSubsystem extends SubsystemBase {
  private final LEDIO io;
  private final LEDInputsAutoLogged inputs = new LEDInputsAutoLogged();

  private BooleanSupplier launchingSupplier = () -> false;
  private BooleanSupplier intakingSupplier = () -> false;
  private BooleanSupplier detectingSupplier = () -> false;
  private BooleanSupplier partySupplier = () -> DriverStation.isTestEnabled();

  private LEDState currentState = LEDState.NOT_ACTIVE;
  private LEDState lastAppliedState = null;
  private String currentAnimationName = "None";

  public LEDSubsystem(LEDIO io) {
    this.io = io;
  }

  public void configureStateSuppliers(
      BooleanSupplier launchingSupplier,
      BooleanSupplier intakingSupplier,
      BooleanSupplier detectingSupplier,
      BooleanSupplier partySupplier) {
    this.launchingSupplier = launchingSupplier;
    this.intakingSupplier = intakingSupplier;
    this.detectingSupplier = detectingSupplier;
    this.partySupplier = partySupplier;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("LEDs", inputs);

    currentState = evaluateState();
    if (currentState != lastAppliedState) {
      applyState(currentState);
      lastAppliedState = currentState;
    }

    Logger.recordOutput("LEDs/State", currentState.name());
    Logger.recordOutput("LEDs/Animation", currentAnimationName);
  }

  public boolean isConnected() {
    return inputs.connected;
  }

  public StatusCode setSolidColor(int red, int green, int blue) {
    return io.setSolidColor(red, green, blue);
  }

  public StatusCode setDefaultColor() {
    return setSolidColor(
        LEDConstants.defaultRed, LEDConstants.defaultGreen, LEDConstants.defaultBlue);
  }

  public StatusCode off() {
    return setSolidColor(0, 0, 0);
  }

  public StatusCode clearAnimations() {
    return io.clearAnimations();
  }

  public StatusCode setControl(ControlRequest request) {
    currentAnimationName = request.getClass().getSimpleName();
    return io.setControl(request);
  }

  public Command setAnimation(ControlRequest request) {
    return Commands.runOnce(() -> setControl(request), this).withName("LED Set Animation");
  }

  private LEDState evaluateState() {
    boolean enabled = DriverStation.isEnabled();
    boolean teleopEnabled = DriverStation.isTeleopEnabled();
    boolean estopped = DriverStation.isEStopped();
    boolean brownedOut = RobotController.isBrownedOut();

    double matchTime = DriverStation.getMatchTime();
    boolean inEndgame =
        teleopEnabled && matchTime >= 0.0 && matchTime <= LEDConstants.endgamePanicThresholdSec;

    if (!inputs.connected || brownedOut) {
      return LEDState.PROBLEM;
    }

    if (estopped) {
      return LEDState.STOP_IT;
    }

    if (inEndgame) {
      return LEDState.ENDGAME_PANIC;
    }

    if (launchingSupplier.getAsBoolean()) {
      return LEDState.LAUNCHING;
    }

    if (intakingSupplier.getAsBoolean()) {
      return LEDState.INTAKING;
    }

    if (detectingSupplier.getAsBoolean()) {
      return LEDState.DETECTING;
    }

    if (partySupplier.getAsBoolean()) {
      return LEDState.PARTY;
    }

    return enabled ? LEDState.ACTIVE : LEDState.NOT_ACTIVE;
  }

  private void applyState(LEDState state) {
    int endLed = Math.max(0, LEDConstants.ledCount - 1);

    switch (state) {
      case PROBLEM -> {
        setSolidColor(LEDConstants.problemRed, LEDConstants.problemGreen, LEDConstants.problemBlue);
        currentAnimationName = "SolidRed";
      }
      case STOP_IT -> {
        setSolidColor(LEDConstants.stopRed, LEDConstants.stopGreen, LEDConstants.stopBlue);
        currentAnimationName = "SolidMagenta";
      }
      case ENDGAME_PANIC -> {
        setControl(new RgbFadeAnimation(0, endLed).withSlot(0));
        currentAnimationName = "RgbFadeEndgame";
      }
      case LAUNCHING -> {
        setSolidColor(
            LEDConstants.launchingRed, LEDConstants.launchingGreen, LEDConstants.launchingBlue);
        currentAnimationName = "SolidGreen";
      }
      case INTAKING -> {
        setSolidColor(
            LEDConstants.intakingRed, LEDConstants.intakingGreen, LEDConstants.intakingBlue);
        currentAnimationName = "SolidAmber";
      }
      case DETECTING -> {
        setSolidColor(
            LEDConstants.detectingRed, LEDConstants.detectingGreen, LEDConstants.detectingBlue);
        currentAnimationName = "SolidBlue";
      }
      case PARTY -> {
        setControl(new RgbFadeAnimation(0, endLed).withSlot(0));
        currentAnimationName = "RgbFadeParty";
      }
      case ACTIVE -> {
        setDefaultColor();
        currentAnimationName = "DefaultSolid";
      }
      case NOT_ACTIVE -> {
        off();
        currentAnimationName = "Off";
      }
    }
  }
}
