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
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import frc2713.lib.drivers.CANDeviceId;
import frc2713.lib.dynamics.MoiUnits;
import frc2713.lib.io.CanCoderConfig;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;
import frc2713.lib.subsystem.TalonFXSubsystemConfig.GeneralControlMode;
import frc2713.lib.util.LoggedTunableBoolean;
import frc2713.lib.util.LoggedTunableMeasure;
import frc2713.lib.util.Util;
import frc2713.robot.GamePieceConstants;

public final class LauncherConstants {

  public static final class Turret {

    public static TalonFXSubsystemConfig config = new TalonFXSubsystemConfig();
    public static CanCoderConfig canCoderConfig = new CanCoderConfig();
    public static Angle staticHubAngle = Degrees.of(0);

    // Turret rotation limits
    public static final double FORWARD_LIMIT_DEGREES = 15.0;
    public static final double REVERSE_LIMIT_DEGREES = -365.0;

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
        ((double) spurGear1Teeth / (double) pinionGearTeeth)
            * ((double) sprocketGearTeeth / (double) sprocketPinionTeeth);

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
      config.generalControlMode = GeneralControlMode.POSITION;
      config.acceptablePositionError = Degrees.of(3);

      config.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      config.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

      // PID gains for Motion Magic
      config.fxConfig.Slot0.kP = Util.modeDependentValue(550.0, 80.0);
      config.fxConfig.Slot0.kI = 0.0;
      config.fxConfig.Slot0.kD = Util.modeDependentValue(8.0, 16.0);
      config.fxConfig.Slot0.kS =
          Util.modeDependentValue(0.23, 0.15); // static friction compensation
      config.fxConfig.Slot0.kV = Util.modeDependentValue(0.0, 0.12); // velocity feedforward
      config.fxConfig.Slot0.kA = Util.modeDependentValue(0.0, 0.01); // acceleration feedforward

      // Motion Magic parameters
      config.fxConfig.MotionMagic.MotionMagicCruiseVelocity = 20.0; // rotations per second
      config.fxConfig.MotionMagic.MotionMagicAcceleration = 10.0; // rotations per second^2
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
              new Rotation3d(0, 0, Math.PI));
    }

    static {
      canCoderConfig.canId =
          new CANDeviceId(53, "canivore"); // CANCoder CAN ID, replace with actual ID
    }

    public static int MODEL_INDEX = 3;
    public static int PARENT_INDEX = 0; // drivetrain

    public static LoggedTunableMeasure<Angle> PIDTestAngleOne =
        new LoggedTunableMeasure<>("Turret/PIDTestAngleOne", Degrees.of(10));
    public static LoggedTunableMeasure<Angle> PIDTestAngleTwo =
        new LoggedTunableMeasure<>("Turret/PIDTestAngleTwo", Degrees.of(-10));
    public static LoggedTunableMeasure<Angle> staticLeftTrench =
        new LoggedTunableMeasure<>("Turret/Turret Static Trench L", Degrees.of(10));
    public static LoggedTunableMeasure<Angle> staticRightTrench =
        new LoggedTunableMeasure<>("Turret/Turret Static Trench R", Degrees.of(-10));
    public static LoggedTunableMeasure<Angle> staticTowerShot =
        new LoggedTunableMeasure<>("Turret/Turret Static Tower", Degrees.of(3));
  }

  public final class Flywheels {

    public static TalonFXSubsystemConfig leaderConfig = new TalonFXSubsystemConfig();
    public static TalonFXSubsystemConfig followerConfig = new TalonFXSubsystemConfig();
    public static MomentOfInertia flywhMomentOfInertia = MoiUnits.PoundSquareInches.of(10.410164);
    public static double gearRatio = 24.0 / 18.0; // 1.33:1 reduction from motor to flywheel

    static {
      leaderConfig.name = "Flywheels";
      leaderConfig.talonCANID = new CANDeviceId(50, "canivore");
      leaderConfig.fxConfig.Slot0.kP = Util.modeDependentValue(30., 3.5);
      leaderConfig.fxConfig.Slot0.kI = 0.0;
      leaderConfig.fxConfig.Slot0.kD = 0.004;
      leaderConfig.fxConfig.Slot0.kS = Util.modeDependentValue(0.15, 2.0);
      leaderConfig.fxConfig.Slot0.kV = 0.12 * gearRatio;
      leaderConfig.fxConfig.CurrentLimits.StatorCurrentLimit = 120.0;
      leaderConfig.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      leaderConfig.fxConfig.CurrentLimits.SupplyCurrentLimit = 70.0;
      leaderConfig.fxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      leaderConfig.simOrientation = ChassisReference.CounterClockwise_Positive;

      leaderConfig.unitToRotorRatio = gearRatio; // 1.33:1 reduction from motor to flywheel
      leaderConfig.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      leaderConfig.fxConfig.Voltage.PeakReverseVoltage = 0;
      leaderConfig.fxConfig.TorqueCurrent.PeakReverseTorqueCurrent = 0;
      leaderConfig.momentOfInertia = flywhMomentOfInertia.times(0.5);
      leaderConfig.useFOC = false; // FOC makes the feedfowrward term units weird
      leaderConfig.tunable = true;
      leaderConfig.fxConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      leaderConfig.generalControlMode = GeneralControlMode.VELOCITY;
      leaderConfig.acceptableVelocityError = RPM.of(100);

      followerConfig.name = "Flywheels Follower";
      followerConfig.talonCANID = new CANDeviceId(51, "canivore");
      followerConfig.unitToRotorRatio = gearRatio; // 1.33:1 reduction from motor to flywheel
      followerConfig.momentOfInertia = flywhMomentOfInertia.times(0.5);
      followerConfig.useFOC = false;
      followerConfig.fxConfig.CurrentLimits = leaderConfig.fxConfig.CurrentLimits;
    }

    public static int MODEL_INDEX = 5;
    public static int PARENT_INDEX = 4;

    public static Transform3d localTransform =
        new Transform3d(
            new Translation3d(Inches.of(-5).in(Meters), 0, Inches.of(2).in(Meters)),
            new Rotation3d(0, Degrees.of(-90).in(Radians), 0));

    public static InterpolatingDoubleTreeMap ballVelocityMap = new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap RPMVelocityMap = new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap ballVelocityAZMap = new InterpolatingDoubleTreeMap();

    public static InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();

    public static Distance WHEEL_DIAMETER = Inches.of(4);
    // How many fuel we can launch per second at max firing rate
    public static double launchRateFuelPerSecond = 9.0;

    // the volume of fuel we're launching per second at max firing rate
    public static double launchRateVolumeInchesCubedPerSecond =
        launchRateFuelPerSecond * GamePieceConstants.Fuel.volumeInchesCubed;

    static {
      // Distance (m) -> Ball Velocity (ft/s)
      ballVelocityMap.put(2.11, 19.19);
      ballVelocityMap.put(6.44, 22.07);
      //   ballVelocityMap.put(1.0, 20.0);
      //   ballVelocityMap.put(1.5, 20.0);
      //   ballVelocityMap.put(2.5, 22.0);
      //   ballVelocityMap.put(3.2, 24.0);
      //   ballVelocityMap.put(4.0, 28.0);
      //   ballVelocityMap.put(5.17, 29.0);
      //   ballVelocityMap.put(5.4, 30.0);

      RPMVelocityMap.put(2.11, 2713.);
      RPMVelocityMap.put(6.44, 3500.);

      //   ballVelocityAZMap.put(1.0, 20.0);
      //   ballVelocityAZMap.put(1.5, 20.0);
      //   ballVelocityAZMap.put(2.5, 22.0);
      //   ballVelocityAZMap.put(3.2, 23.0);
      //   ballVelocityAZMap.put(4.0, 26.0);
      //   ballVelocityAZMap.put(5.17, 29.0);
      //   ballVelocityAZMap.put(5.4, 30.0);
    }

    static {
      // Ball Velocity (ft/s) -> RPM (rpm)
      rpmMap.put(17.69, 2500.);
      rpmMap.put(20.19, 3000.);
      rpmMap.put(22.07, 3500.);
      rpmMap.put(24.67, 4000.);
      rpmMap.put(25.52, 4500.);
    }

    public static LoggedTunableMeasure<AngularVelocity> idleVelocity =
        new LoggedTunableMeasure<>("Flywheels/Idle Velocity", RPM.of(300));
    public static LoggedTunableMeasure<AngularVelocity> PIDTest =
        new LoggedTunableMeasure<>("Flywheels/PIDTest", RPM.of(4000));
    public static LoggedTunableMeasure<AngularVelocity> staticRightLeftTrench =
        new LoggedTunableMeasure<>("Flywheels/Flywheels Static Trench", RPM.of(3150));
    public static LoggedTunableMeasure<AngularVelocity> staticHubVelocity =
        new LoggedTunableMeasure<>("Flywheels/Flywheels Static Hub", RotationsPerSecond.of(20));
    public static LoggedTunableMeasure<AngularVelocity> staticTowerVelocity =
        new LoggedTunableMeasure<AngularVelocity>("Flywheels/Flywheels Static Tower", RPM.of(3000));
  }

  public final class Hood {

    public static TalonFXSubsystemConfig config = new TalonFXSubsystemConfig();

    public static final Angle minAngle = Degrees.of(0);
    public static final Angle maxAngle = Degrees.of(30);

    // 8t pinion to 20t gear, 13t gear to  30t gear, 10t gear to 146 sector gear
    public static double gearRatio = ((20 / 8.0) * (30.0 / 13.0) * (146.0 / 10.0));

    public static final Angle retractedPosition = Degrees.of(0);

    static {
      config.name = "Hood";
      config.talonCANID = new CANDeviceId(54, "canivore"); // Example CAN ID, replace with actual ID

      // PID gains for Motion Magic
      config.fxConfig.Slot0.kP = 600.0;
      config.fxConfig.Slot0.kI = 254.0;
      config.fxConfig.Slot0.kD = 30.0;
      config.fxConfig.Slot0.kS = 5.0; // static friction compensation
      config.fxConfig.Slot0.kV = 0.092 * gearRatio; // velocity feedforward
      config.fxConfig.Slot0.kA = 0.0;

      config.tunable = true;
      config.generalControlMode = GeneralControlMode.POSITION;
      config.acceptablePositionError = Degrees.of(2);

      config.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      config.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      config.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = maxAngle.in(Rotations);
      config.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = minAngle.in(Rotations);

      config.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      config.fxConfig.CurrentLimits.StatorCurrentLimit = 30;
      config.fxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      config.fxConfig.CurrentLimits.SupplyCurrentLimit = 20;

      // Motion Magic parameters
      config.fxConfig.MotionMagic.MotionMagicCruiseVelocity = 5; // rotations per second
      config.fxConfig.MotionMagic.MotionMagicAcceleration = 500.0; // rotations per second^2
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

    public static InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap angleForAZMap = new InterpolatingDoubleTreeMap();

    static {
      // Distance (m) -> Hood Pitch (Degrees)
      angleMap.put(5.486, 28.74);
      angleMap.put(4.786, 26.0);
      angleMap.put(4.175, 25.0);
    }
    // 5.486 2000 28.74
    // 4.786 1750 26

    static {
      // Distance (m) -> Hood Pitch (Degrees)
      angleForAZMap.put(5.486, 28.74);
      angleForAZMap.put(4.786, 26.0);
      angleForAZMap.put(4.175, 25.0);
    }

    public static LoggedTunableMeasure<Angle> staticTowerAngle =
        new LoggedTunableMeasure<Angle>("Hood/Hood Static Tower", Degrees.of(25));
    public static LoggedTunableMeasure<Angle> staticRightLeftTrenchAngle =
        new LoggedTunableMeasure<Angle>("Hood/Hood Static Trench", Degrees.of(25));
    public static LoggedTunableMeasure<Angle> staticHubAngle =
        new LoggedTunableMeasure<Angle>("Hood/Hood Static Hub", Degrees.of(25));
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
