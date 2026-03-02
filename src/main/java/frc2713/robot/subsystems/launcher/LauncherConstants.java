package frc2713.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import frc2713.lib.drivers.CANDeviceId;
import frc2713.lib.dynamics.MoiUnits;
import frc2713.lib.io.CanCoderConfig;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;
import frc2713.lib.util.LoggedTunableBoolean;
import frc2713.lib.util.LoggedTunableMeasure;

public final class LauncherConstants {

  public static final class Turret {

    public static TalonFXSubsystemConfig config = new TalonFXSubsystemConfig();
    public static CanCoderConfig canCoderConfig = new CanCoderConfig();
    public static Angle acceptableError = Degrees.of(3);
    public static Angle staticHubAngle = Degrees.of(0);

    // Turret rotation limits
    public static final double FORWARD_LIMIT_DEGREES = 210.0;
    public static final double REVERSE_LIMIT_DEGREES = -210.0;

    public static final Angle forwardSoftLimit = Degrees.of(FORWARD_LIMIT_DEGREES);
    public static final Angle reverseSoftLimit = Degrees.of(REVERSE_LIMIT_DEGREES);

    // Gear tooth counts for calculating overall gear ratio
    public static final int pinionGearTeeth = 15;
    public static final int spurGear1Teeth = 26;
    public static final int sprocketPinionTeeth = 16;
    public static final int sprocketGearTeeth = 224;

    // Overall gear ratio from motor rotations to turret rotations
    // motor has an absolute encoder, so this can be encoder 1
    public static final double motorToTurretGearRatio =
        (spurGear1Teeth / pinionGearTeeth) * (sprocketGearTeeth / sprocketPinionTeeth);

    // Gear ratio from motor rotations to encoder rotations (encoder is after the first stage
    // reduction)
    public static final double motorToEncoderGearRatio = spurGear1Teeth / pinionGearTeeth;

    // Gear ratio from encoder rotations to turret rotations (encoder is after the first stage
    // reduction)
    public static final double encoderToTurretGearRatio = sprocketGearTeeth / sprocketPinionTeeth;

    static {
      config.name = "Turret";
      config.talonCANID = new CANDeviceId(52, "canivore");
      config.tunable = true; // Enable tunable gains for Motion Magic

      // PID gains for Motion Magic
      config.fxConfig.Slot0.kP = 80.0;
      config.fxConfig.Slot0.kI = 0.0;
      config.fxConfig.Slot0.kD = 16.0;
      config.fxConfig.Slot0.kS = 0.15; // static friction compensation
      config.fxConfig.Slot0.kV = 0.12; // velocity feedforward
      config.fxConfig.Slot0.kA = 0.01;

      config.fxConfig.Feedback.SensorToMechanismRatio = 1 / motorToTurretGearRatio;

      // Motion Magic parameters
      config.fxConfig.MotionMagic.MotionMagicCruiseVelocity = 5.0; // rotations per second
      config.fxConfig.MotionMagic.MotionMagicAcceleration = 15.0; // rotations per second^2
      config.fxConfig.MotionMagic.MotionMagicJerk = 100; // limit jerk for smooth motion

      // Gear ratio: motor rotations per turret rotation = GEAR_1/GEAR_0 = 120/60 = 2.0
      config.unitToRotorRatio = motorToTurretGearRatio;
      config.momentOfInertia = MoiUnits.PoundSquareInches.of(522.908341);

      config.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      config.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      config.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
          forwardSoftLimit.in(Rotations);
      config.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
          reverseSoftLimit.in(Rotations);

      config.initialTransform =
          new Transform3d(
              new Translation3d(
                  Inches.of(0.5).in(Meters),
                  Inches.of(0.5).in(Meters),
                  Inches.of(18.484119).in(Meters)),
              new Rotation3d(0, 0, 0));
    }

    static {
      canCoderConfig.canId =
          new CANDeviceId(53, "canivore"); // CANCoder CAN ID, replace with actual ID
    }

    public static int MODEL_INDEX = 3;
    public static int PARENT_INDEX = 0; // drivetrain
  }

  public final class Flywheels {
    public static final LoggedTunableMeasure<AngularVelocity> PIDTest =
        new LoggedTunableMeasure<>("Flywheels/PIDTest", RPM.of(2000));
    public static final LoggedTunableMeasure<AngularVelocity> launchVelocity =
        new LoggedTunableMeasure<>("Flywheels/launchVelocity", RPM.of(2500));

    public static TalonFXSubsystemConfig leaderConfig = new TalonFXSubsystemConfig();
    public static TalonFXSubsystemConfig followerConfig = new TalonFXSubsystemConfig();
    public static MomentOfInertia flywhMomentOfInertia = MoiUnits.PoundSquareInches.of(10.410164);
    public static double gearRatio = 24.0 / 18.0; // 1.33:1 overdrive from motor to flywheel

    static {
      leaderConfig.name = "Flywheels";
      leaderConfig.talonCANID = new CANDeviceId(50, "canivore");
      leaderConfig.fxConfig.Slot0.kP = 0.5; // Util.modeDependentValue(3.0, 3.5);
      leaderConfig.fxConfig.Slot0.kI = 0.0;
      leaderConfig.fxConfig.Slot0.kD = 0.0;
      leaderConfig.fxConfig.Slot0.kS = 2.0;
      leaderConfig.fxConfig.Slot0.kV = 0.12 * (1.0 / gearRatio);
      leaderConfig.fxConfig.CurrentLimits.StatorCurrentLimit = 120.0;
      leaderConfig.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      leaderConfig.fxConfig.CurrentLimits.SupplyCurrentLimit = 70.0;
      leaderConfig.fxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

      leaderConfig.unitToRotorRatio = gearRatio; // 1.33:1 reduction from motor to flywheel
      leaderConfig.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      leaderConfig.fxConfig.MotorOutput.PeakReverseDutyCycle = 0;
      leaderConfig.momentOfInertia = flywhMomentOfInertia.times(0.5);
      leaderConfig.useFOC = false; // FOC makes the feedfowrward term units weird
      leaderConfig.tunable = true;
      leaderConfig.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      followerConfig.name = "Flywheels Follower";
      followerConfig.talonCANID = new CANDeviceId(51, "canivore");
      followerConfig.unitToRotorRatio = gearRatio; // 1.33:1 reduction from motor to flywheel
      followerConfig.momentOfInertia = flywhMomentOfInertia.times(0.5);
      followerConfig.useFOC = false;
      followerConfig.fxConfig.CurrentLimits.StatorCurrentLimit = 120.0;
      followerConfig.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      followerConfig.fxConfig.CurrentLimits.SupplyCurrentLimit = 70.0;
      followerConfig.fxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    }

    public static int MODEL_INDEX = 5;
    public static int PARENT_INDEX = 4;

    public static AngularVelocity acceptableError = RPM.of(35);
    public static AngularVelocity idleVelocity = RotationsPerSecond.of(20);

    public static Transform3d localTransform =
        new Transform3d(
            new Translation3d(Inches.of(-5).in(Meters), 0, Inches.of(2).in(Meters)),
            new Rotation3d(0, Degrees.of(-90).in(Radians), 0));

    public static AngularVelocity staticHubVelocity = RotationsPerSecond.of(20);
    public static AngularVelocity staticTowerVelocity = RPM.of(1500);
    public static InterpolatingDoubleTreeMap velocityMap = new InterpolatingDoubleTreeMap();

    public static Distance WHEEL_DIAMETER = Inches.of(4);
    // How many fuel we can launch per second at max firing rate
    public static double launchRateFuelPerSecond = 9.0;

    static {
      // Distance (m) -> Ball Velocity (ft/s)
      velocityMap.put(1.0, 20.0);
      velocityMap.put(1.5, 20.0);
      velocityMap.put(2.5, 22.0);
      velocityMap.put(3.2, 23.0);
      velocityMap.put(4.0, 26.0);
      velocityMap.put(5.17, 29.0);
      velocityMap.put(5.4, 30.0);
    }
  }

  public final class Hood {

    public static TalonFXSubsystemConfig config = new TalonFXSubsystemConfig();
    public static Angle acceptableError = Degrees.of(5);

    // 9 tooth pinion to 20 tooth gear, 16 tooth gear to 38 tooth gear, 10 tooth gear to 124 tooth
    // gear for total reduction of 0.0306
    public static double gearRatio = 1 / ((8.0 / 52.0) * (16.0 / 38.0) * (10.0 / 124.0));

    public static final Angle retractedPosition = Degrees.of(0);

    static {
      config.name = "Hood";
      config.talonCANID = new CANDeviceId(54, "canivore"); // Example CAN ID, replace with actual ID

      // PID gains for Motion Magic
      config.fxConfig.Slot0.kP = 0.0;
      config.fxConfig.Slot0.kI = 0.0;
      config.fxConfig.Slot0.kD = 0.0;
      config.fxConfig.Slot0.kS = 0.0; // static friction compensation
      config.fxConfig.Slot0.kV = 0.092 * gearRatio; // velocity feedforward
      config.fxConfig.Slot0.kA = 0.0;

      config.tunable = true;

      config.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      config.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      config.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(75);
      config.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(0);

      config.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      config.fxConfig.CurrentLimits.StatorCurrentLimit = 10;
      config.fxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      config.fxConfig.CurrentLimits.SupplyCurrentLimit = 10;

      // Motion Magic parameters
      config.fxConfig.MotionMagic.MotionMagicCruiseVelocity = 2.0; // rotations per second
      config.fxConfig.MotionMagic.MotionMagicAcceleration = 4.0; // rotations per second^2
      config.fxConfig.MotionMagic.MotionMagicJerk = 0; // no jerk limit

      config.unitToRotorRatio = gearRatio;
      config.momentOfInertia = MoiUnits.PoundSquareInches.of(38.979757);

      config.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      config.initialTransform =
          new Transform3d(
              new Translation3d(Inches.of(2.452807).in(Meters), 0, Inches.of(1.026032).in(Meters)),
              new Rotation3d());
    }

    public static int MODEL_INDEX = 4;
    public static int PARENT_INDEX = 3; // turret

    public static LoggedTunableMeasure<Angle> staticHubAngle =
        new LoggedTunableMeasure<Angle>("Hood/Static Hub", Degrees.of(30));

    public static InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();

    static {
      // Distance (m) -> Hood Pitch (Degrees)
      angleMap.put(0.9, 9.0);
      angleMap.put(1.0, 11.0);
      angleMap.put(1.5, 18.0);
      angleMap.put(2.0, 24.0);
      angleMap.put(3.0, 28.0);
      angleMap.put(4.0, 30.0);
      angleMap.put(4.5, 30.0);
      angleMap.put(5.0, 30.0);
    }
  }

  public static LoggedTunableMeasure<Time> otfLinearProjectionSeconds =
      new LoggedTunableMeasure<Time>(
          "LaunchingSolutionManager/time_to_project_lin", Seconds.of(0.5));
  public static LoggedTunableMeasure<Time> otfAngularProjectionSeconds =
      new LoggedTunableMeasure<Time>(
          "LaunchingSolutionManager/time_to_project_ang", Seconds.of(0.5));
  public static LoggedTunableBoolean otfFutureProjectionEnabled =
      new LoggedTunableBoolean("LaunchingSolutionManager/projection_enabled", true);
}
