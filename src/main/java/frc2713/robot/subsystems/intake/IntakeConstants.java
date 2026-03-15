package frc2713.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.DifferentialMotorConstants;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import frc2713.lib.drivers.CANDeviceId;
import frc2713.lib.dynamics.MoiUnits;
import frc2713.lib.subsystem.DifferentialSubsystemConfig;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;
import frc2713.lib.subsystem.TalonFXSubsystemConfig.GeneralControlMode;
import frc2713.lib.util.LoggedTunableMeasure;
import frc2713.lib.util.LoggedTunableNumber;
import frc2713.lib.util.Util;
import java.util.function.Supplier;

public final class IntakeConstants {

  public static final class Roller {

    public static TalonFXSubsystemConfig leaderConfig = new TalonFXSubsystemConfig();
    public static TalonFXSubsystemConfig followerConfig = new TalonFXSubsystemConfig();

    public static final double gearRatio =
        24.0 / 12.0; // 12 tooth pinion to 24 tooth gear for 0.5 reduction
    public static final MomentOfInertia rollersMomentOfInertia =
        MoiUnits.PoundSquareInches.of(0.295439).times(2); // Low MOI for fast-spinning rollers

    static {
      leaderConfig.name = "Intake Rollers";
      leaderConfig.talonCANID = new CANDeviceId(42); // Example CAN ID, replace with actual ID
      leaderConfig.unitToRotorRatio =
          gearRatio; // 12 tooth pinion to 24 tooth gear for 0.5 reduction
      leaderConfig.momentOfInertia = rollersMomentOfInertia.times(0.5);
      leaderConfig.useFOC = true;
      leaderConfig.motor = DCMotor.getKrakenX44Foc(1);
      leaderConfig.simOrientation = ChassisReference.CounterClockwise_Positive;
      leaderConfig.tunable = true;

      leaderConfig.fxConfig.Slot0.kP = Util.modeDependentValue(0.75, 3.5);
      leaderConfig.fxConfig.Slot0.kI = 0.0;
      leaderConfig.fxConfig.Slot0.kD = 0.0;
      leaderConfig.fxConfig.Slot0.kS = 2.0;
      leaderConfig.fxConfig.Slot0.kV = 0.12 * gearRatio;
      leaderConfig.fxConfig.CurrentLimits.StatorCurrentLimit = 120.0;
      leaderConfig.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      leaderConfig.fxConfig.CurrentLimits.SupplyCurrentLimit = 70.0;
      leaderConfig.fxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

      leaderConfig.fxConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.05;

      followerConfig.name = "Intake Rollers Follower";
      followerConfig.talonCANID = new CANDeviceId(43); // Example CAN ID, replace with actual ID
      followerConfig.unitToRotorRatio =
          gearRatio; // 12 tooth pinion to 24 tooth gear for 0.5 reduction
      followerConfig.momentOfInertia = rollersMomentOfInertia.times(0.5);
      followerConfig.useFOC = true;
      followerConfig.motor = DCMotor.getKrakenX44Foc(1);
      followerConfig.fxConfig.CurrentLimits = leaderConfig.fxConfig.CurrentLimits;
    }

    public static final AngularVelocity freeSpeed =
        RadiansPerSecond.of(leaderConfig.motor.freeSpeedRadPerSec).div(gearRatio);

    public static Voltage outtakeVoltageDesired = Volts.of(-5.0);
    public static LoggedTunableMeasure<AngularVelocity> intakeSpeed =
        new LoggedTunableMeasure<>("Intake Rollers/Intake Speed", freeSpeed.times(0.8));
  }

  public static final class Extension {

    public static TalonFXSubsystemConfig config = new TalonFXSubsystemConfig();
    public static DifferentialSubsystemConfig differentialConfig =
        new DifferentialSubsystemConfig();

    // Ratios
    public static final double averageGearRatio = 60.0 / 8.0;
    public static final Distance sprocketPitchDiameter =
        Inches.of(1.273); // Diameter of the circle formed by the center of the sprocket teeth

    // Dynamics
    public static final Mass movingMass = Pounds.of(11.75);
    public static final LinearVelocity cruiseVelocity = InchesPerSecond.of(24);

    // Dimensions
    public static final Distance height = Inches.of(15.5);
    public static final Distance width = Inches.of(30.0);
    public static final double volumePerInch = height.in(Inches) * width.in(Inches);

    // Soft limits (in rotations of the mechanism)
    public static final double forwardSoftLimit =
        11.5 / (sprocketPitchDiameter.in(Inches) * Math.PI); // ~12 inches max extension
    public static final double reverseSoftLimit =
        0
            / (sprocketPitchDiameter.in(Inches)
                * Math.PI); // -0.5 inches to allow slight over-retraction

    // A supplier to prevent changes by subsystem from propagating
    public static final Supplier<MotionMagicConfigs> motionMagicGains =
        () ->
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(
                    RotationsPerSecond.of(
                        Util.modeDependentValue(300, 30))) // target crusing vel rps
                .withMotionMagicAcceleration(
                    RotationsPerSecondPerSecond.of(Util.modeDependentValue(1000, 10)))
                .withMotionMagicJerk(0);

    static {
      config.unitToRotorRatio = averageGearRatio;
      config.metersPerRotation = sprocketPitchDiameter.in(Meters) * Math.PI;
      var avgGains =
          new Slot0Configs()
              .withKP(Util.modeDependentValue(5, 8))
              .withKI(0)
              .withKD(Util.modeDependentValue(0., 0.4))
              .withKS(0)
              .withKV(0.092 * averageGearRatio)
              .withKA(0);

      config.name = "Intake Extension";
      config.talonCANID = new CANDeviceId(40); // Only used for sim, no real CAN ID
      config.fxConfig.Slot0 = avgGains;
      config.fxConfig.MotionMagic = motionMagicGains.get();
      config.motor = DCMotor.getKrakenX44(1);
      config.simOrientation = ChassisReference.CounterClockwise_Positive;

      // MOI = m*r^2, where r is the radius to the center of mass (half the pitch diameter)
      config.momentOfInertia =
          MoiUnits.PoundSquareInches.of(
              movingMass.in(Pounds) * Math.pow(sprocketPitchDiameter.in(Inches) / 2.0, 2));
      config.tunable = true;

      config.generalControlMode = GeneralControlMode.POSITION;
      TalonFXSubsystemConfig.setLinearAcceptableError(
          config, config.metersPerRotation, Inches.of(0.2));

      // Differential configuration
      differentialConfig.name = config.name;
      differentialConfig.leaderCANID = new CANDeviceId(40, "canivore"); // Leader motor CAN ID
      differentialConfig.followerCANID = new CANDeviceId(41, "canivore"); // Follower motor CAN ID

      // Average axis gains typically go in Slot 0
      differentialConfig.averageGains = avgGains;

      // Difference axis gains typically go in Slot 1
      differentialConfig.differenceGains =
          new Slot1Configs().withKP(1).withKI(0).withKD(0).withKS(0.).withKV(0.);

      differentialConfig.averageGearRatio = averageGearRatio;
      differentialConfig.differenceGearRatio = 1.0;
      differentialConfig.motorAlignment = MotorAlignmentValue.Opposed;
      differentialConfig.closedLoopRate = 200.0;
      differentialConfig.followerUsesCommonLeaderConfigs = true;
      differentialConfig.tunable = true;

      // Leader initial configs
      differentialConfig.leaderConfig =
          new TalonFXConfiguration()
              .withMotorOutput(
                  new MotorOutputConfigs()
                      .withNeutralMode(NeutralModeValue.Coast)
                      .withInverted(InvertedValue.Clockwise_Positive))
              .withCurrentLimits(
                  new CurrentLimitsConfigs()
                      .withStatorCurrentLimit(80.0)
                      .withStatorCurrentLimitEnable(true))
              .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(averageGearRatio))
              .withSoftwareLimitSwitch(
                  new SoftwareLimitSwitchConfigs()
                      .withForwardSoftLimitEnable(true)
                      .withForwardSoftLimitThreshold(forwardSoftLimit)
                      .withReverseSoftLimitEnable(true)
                      .withReverseSoftLimitThreshold(reverseSoftLimit))
              .withClosedLoopGeneral(
                  new ClosedLoopGeneralConfigs()
                      // differential mechanism is not continuous on the difference axis
                      .withDifferentialContinuousWrap(false))
              .withSlot0(differentialConfig.averageGains)
              .withSlot1(differentialConfig.differenceGains)
              .withMotionMagic(config.fxConfig.MotionMagic);

      // Follower initial configs
      differentialConfig.followerConfig =
          new TalonFXConfiguration()
              .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(averageGearRatio));

      // Differential mechanism constants
      differentialConfig.differentialConstants =
          new DifferentialMotorConstants<TalonFXConfiguration>()
              .withLeaderId(differentialConfig.leaderCANID.getDeviceNumber())
              .withFollowerId(differentialConfig.followerCANID.getDeviceNumber())
              .withCANBusName(differentialConfig.leaderCANID.getBus())
              .withAlignment(differentialConfig.motorAlignment)
              .withSensorToDifferentialRatio(differentialConfig.differenceGearRatio)
              .withClosedLoopRate(differentialConfig.closedLoopRate)
              .withLeaderInitialConfigs(differentialConfig.leaderConfig)
              .withFollowerInitialConfigs(differentialConfig.followerConfig)
              .withFollowerUsesCommonLeaderConfigs(
                  differentialConfig.followerUsesCommonLeaderConfigs);
    }

    public static LoggedTunableMeasure<Distance> extendedPosition =
        new LoggedTunableMeasure<>(config.name + "/Extended Position", Inches.of(11.5));
    public static LoggedTunableMeasure<Distance> pidTestPosition =
        new LoggedTunableMeasure<>(config.name + "/PID Test Position", Inches.of(0));
    public static LoggedTunableMeasure<Distance> retractedPosition =
        new LoggedTunableMeasure<>(config.name + "/Retracted Position", Inches.of(0));
    public static final LoggedTunableMeasure<LinearVelocity> retractCruiseVelocity =
        new LoggedTunableMeasure<>(config.name + "/Retract Cruise Velocity", InchesPerSecond.of(4));
    public static final LoggedTunableNumber fuelPressureScalingFactor =
        new LoggedTunableNumber(config.name + "/Fuel Pressure Scaling", 1.1);

    public static int MODEL_INDEX = 1;
    public static int PARENT_INDEX = 0; // drivetrain
  }
}
