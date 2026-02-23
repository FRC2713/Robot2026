package frc2713.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.DifferentialMotorConstants;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc2713.lib.drivers.CANDeviceId;
import frc2713.lib.subsystem.DifferentialSubsystemConfig;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;
import frc2713.lib.util.LoggedTunableMeasure;

public final class IntakeConstants {

  public static final class Roller {

    public static TalonFXSubsystemConfig config = new TalonFXSubsystemConfig();

    static {
      config.name = "Intake Roller";
      config.talonCANID = new CANDeviceId(9); // Example CAN ID, replace with actual ID
      config.unitToRotorRatio = 1.0; // 1:1 ratio
      config.momentOfInertia = 0.001; // Low MOI for fast-spinning rollers
    }

    public static Voltage intakeVoltageDesired = Volts.of(5.0);
    public static Voltage outtakeVoltageDesired = Volts.of(-5.0);
  }

  public static final class Extension {

    public static TalonFXSubsystemConfig config = new TalonFXSubsystemConfig();
    public static DifferentialSubsystemConfig differentialConfig =
        new DifferentialSubsystemConfig();

    static {
      config.name = "Intake Extension";
      config.talonCANID = new CANDeviceId(8); // Example CAN ID, replace with actual ID
      config.fxConfig.Slot0.kP = 65.0;
      config.fxConfig.Slot0.kI = 0.0;
      config.fxConfig.Slot0.kD = 15.0;
      config.fxConfig.Slot0.kS = 0.125;
      config.fxConfig.Slot0.kV = 0.11;
      config.fxConfig.Slot0.kA = 0.0;
      config.fxConfig.MotionMagic.MotionMagicCruiseVelocity = 4.0; // target crusing vel rps
      config.fxConfig.MotionMagic.MotionMagicAcceleration = 6.0;
      config.fxConfig.MotionMagic.MotionMagicJerk = 0;

      config.unitToRotorRatio = 1.0; // assumes 1:1 gearbox
      config.unitRotationsPerMeter =
          1.0 / Inches.of(12).in(Meters); // assumes 1 ft of travel per rotation

      // Moment of inertia for sim - reasonable for light linear mechanism
      config.momentOfInertia = 0.01;
      config.tunable = true;

      // Differential configuration
      differentialConfig.name = "Intake Extension Differential";
      differentialConfig.leaderCANID = new CANDeviceId(10); // Leader motor CAN ID
      differentialConfig.followerCANID = new CANDeviceId(11); // Follower motor CAN ID

      // Average axis gains typically go in Slot 0
      differentialConfig.averageGains =
          new Slot0Configs()
              .withKP(20)
              .withKI(0)
              .withKD(0.1)
              .withKG(0.2)
              .withKS(0.1)
              .withKV(0.36)
              .withKA(0)
              .withGravityType(GravityTypeValue.Arm_Cosine);

      // Difference axis gains typically go in Slot 1
      differentialConfig.differenceGains =
          new Slot1Configs().withKP(30).withKI(0).withKD(0.1).withKS(0.1).withKV(0.72);

      differentialConfig.averageGearRatio = 3.0;
      differentialConfig.differenceGearRatio = 2.0;
      differentialConfig.motorAlignment = MotorAlignmentValue.Aligned;
      differentialConfig.closedLoopRate = 200.0;
      differentialConfig.followerUsesCommonLeaderConfigs = true;
      differentialConfig.tunable = true;

      // Leader initial configs
      differentialConfig.leaderConfig =
          new TalonFXConfiguration()
              .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
              .withCurrentLimits(
                  new CurrentLimitsConfigs()
                      .withStatorCurrentLimit(80.0)
                      .withStatorCurrentLimitEnable(true))
              .withFeedback(
                  new FeedbackConfigs()
                      .withSensorToMechanismRatio(differentialConfig.averageGearRatio))
              .withClosedLoopGeneral(
                  new ClosedLoopGeneralConfigs()
                      // differential mechanism is continuous on the difference axis
                      .withDifferentialContinuousWrap(true))
              .withSlot0(differentialConfig.averageGains)
              .withSlot1(differentialConfig.differenceGains)
              .withMotionMagic(
                  new MotionMagicConfigs()
                      .withMotionMagicCruiseVelocity(80)
                      .withMotionMagicAcceleration(320));

      // Follower initial configs
      differentialConfig.followerConfig =
          new TalonFXConfiguration()
              .withFeedback(
                  new FeedbackConfigs()
                      .withSensorToMechanismRatio(differentialConfig.averageGearRatio));

      // Differential mechanism constants
      differentialConfig.differentialConstants =
          new DifferentialMotorConstants<TalonFXConfiguration>()
              .withLeaderId(differentialConfig.leaderCANID.getDeviceNumber())
              .withFollowerId(differentialConfig.followerCANID.getDeviceNumber())
              .withAlignment(differentialConfig.motorAlignment)
              .withSensorToDifferentialRatio(differentialConfig.differenceGearRatio)
              .withClosedLoopRate(differentialConfig.closedLoopRate)
              .withLeaderInitialConfigs(differentialConfig.leaderConfig)
              .withFollowerInitialConfigs(differentialConfig.followerConfig)
              .withFollowerUsesCommonLeaderConfigs(
                  differentialConfig.followerUsesCommonLeaderConfigs);
    }

    public static LoggedTunableMeasure<Distance> extendedPosition =
        new LoggedTunableMeasure<>(config.name + "/Extended Position", Inches.of(12.0));
    public static Distance retractedPosition = Inches.of(0);
    public static int MODEL_INDEX = 1;
    public static int PARENT_INDEX = 0; // drivetrain
  }
}
