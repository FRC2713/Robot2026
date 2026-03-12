package frc2713.lib.util;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.controller.PIDController;
import frc2713.lib.io.AdvantageScopePathBuilder;
import java.util.function.BiConsumer;
import lombok.Getter;

/**
 * Helper that exposes tunable entries for a full {@link Slot0Configs} and {@link
 * MotionMagicConfigs} instance.
 */
public class LoggedTunableGains {
  @Getter private final LoggedTunableNumber P;
  @Getter private final LoggedTunableNumber I;
  @Getter private final LoggedTunableNumber D;
  @Getter private final LoggedTunableNumber S;
  @Getter private final LoggedTunableNumber V;
  @Getter private final LoggedTunableNumber A;
  @Getter private final LoggedTunableNumber G;
  @Getter private final LoggedTunableNumber motionMagicCruiseVelocity;
  @Getter private final LoggedTunableNumber motionMagicAcceleration;
  @Getter private final LoggedTunableNumber motionMagicJerk;
  @Getter private final LoggedTunableNumber motionMagicExpo_V;
  @Getter private final LoggedTunableNumber motionMagicExpo_A;

  private final Slot0Configs slot0Ref;
  private final MotionMagicConfigs motionMagicRef;

  /**
   * Create tunables for the provided Slot0Configs. The helper will write values back into the
   * provided Slot0Configs when changes are detected via {@link #ifChanged(int, BiConsumer)}.
   *
   * @param name base dashboard key (use subsystem name)
   * @param gains Slot0Configs instance to mirror and update
   * @param motionMagic MotionMagicConfigs instance to mirror and update
   */
  public LoggedTunableGains(String name, Slot0Configs gains, MotionMagicConfigs motionMagic) {
    AdvantageScopePathBuilder pb = new AdvantageScopePathBuilder(name + "/Gains");
    this.P = new LoggedTunableNumber(pb.makePath("P"), gains.kP);
    this.I = new LoggedTunableNumber(pb.makePath("I"), gains.kI);
    this.D = new LoggedTunableNumber(pb.makePath("D"), gains.kD);
    this.S = new LoggedTunableNumber(pb.makePath("FF", "S"), gains.kS);
    this.V = new LoggedTunableNumber(pb.makePath("FF", "V"), gains.kV);
    this.A = new LoggedTunableNumber(pb.makePath("FF", "A"), gains.kA);
    this.G = new LoggedTunableNumber(pb.makePath("FF", "G"), gains.kG);

    this.motionMagicCruiseVelocity =
        new LoggedTunableNumber(
            pb.makePath("MM", "CruiseVelocity"), motionMagic.MotionMagicCruiseVelocity);
    this.motionMagicAcceleration =
        new LoggedTunableNumber(
            pb.makePath("MM", "Acceleration"), motionMagic.MotionMagicAcceleration);
    this.motionMagicJerk =
        new LoggedTunableNumber(pb.makePath("MM", "Jerk"), motionMagic.MotionMagicJerk);
    this.motionMagicExpo_V =
        new LoggedTunableNumber(pb.makePath("MM", "Expo", "V"), motionMagic.MotionMagicExpo_kV);
    this.motionMagicExpo_A =
        new LoggedTunableNumber(pb.makePath("MM", "Expo", "A"), motionMagic.MotionMagicExpo_kA);
    this.slot0Ref = gains;
    this.motionMagicRef = motionMagic;
  }

  /**
   * Runs the provided action when any of the tunables have changed. The Slot0Configs and
   * MotionMagicConfigs instance passed to the constructor will be updated before the action runs.
   *
   * @param id unique id for the caller (use hashCode())
   * @param action action to run after the Slot0Configs and MotionMagicConfigs have been updated
   */
  public void ifChanged(int id, BiConsumer<Slot0Configs, MotionMagicConfigs> action) {
    LoggedTunableNumber.ifChanged(
        id,
        (double[] values) -> {
          // values order: P, I, D, S, V, A, G, motionMagicCruiseVelocity, motionMagicAcceleration,
          // motionMagicJerk, motionMagicExpo_V, motionMagicExpo_A
          if (values.length >= 1) slot0Ref.kP = values[0];
          if (values.length >= 2) slot0Ref.kI = values[1];
          if (values.length >= 3) slot0Ref.kD = values[2];
          if (values.length >= 4) slot0Ref.kS = values[3];
          if (values.length >= 5) slot0Ref.kV = values[4];
          if (values.length >= 6) slot0Ref.kA = values[5];
          if (values.length >= 7) slot0Ref.kG = values[6];
          if (values.length >= 8) motionMagicRef.MotionMagicCruiseVelocity = values[7];
          if (values.length >= 9) motionMagicRef.MotionMagicAcceleration = values[8];
          if (values.length >= 10) motionMagicRef.MotionMagicJerk = values[9];
          if (values.length >= 11) motionMagicRef.MotionMagicExpo_kV = values[10];
          if (values.length >= 12) motionMagicRef.MotionMagicExpo_kA = values[11];
          action.accept(slot0Ref, motionMagicRef);
        },
        P,
        I,
        D,
        S,
        V,
        A,
        G,
        motionMagicCruiseVelocity,
        motionMagicAcceleration,
        motionMagicJerk,
        motionMagicExpo_V,
        motionMagicExpo_A);
  }

  public PIDController createPIDController() {
    return new PIDController(this.P.getAsDouble(), this.I.getAsDouble(), this.D.getAsDouble());
  }

  public PIDController createAngularPIDController() {
    var pid = new PIDController(this.P.getAsDouble(), this.I.getAsDouble(), this.D.getAsDouble());
    pid.enableContinuousInput(-Math.PI, Math.PI);
    return pid;
  }
}
