package frc2713.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc2713.lib.drivers.CANDeviceId;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;
import frc2713.lib.util.LoggedTunableBoolean;
import frc2713.lib.util.LoggedTunableMeasure;
import frc2713.robot.subsystems.launcher.turretIO.TurretSubsystemConfig;

public final class LauncherConstants {

  public static final class Turret {

    public static TurretSubsystemConfig config = new TurretSubsystemConfig();
    public static Angle acceptableError = Degrees.of(3);
    public static Angle staticHubAngle = Degrees.of(0);

    static {
      config.name = "Turret";
      config.talonCANID = new CANDeviceId(12);
      config.canCoderCANID = new CANDeviceId(13); // CANCoder CAN ID, replace with actual ID
      config.tunable = true; // Enable tunable gains for Motion Magic

      // PID gains for Motion Magic
      config.fxConfig.Slot0.kP = 80.0;
      config.fxConfig.Slot0.kI = 0.0;
      config.fxConfig.Slot0.kD = 16.0;
      config.fxConfig.Slot0.kS = 0.15; // static friction compensation
      config.fxConfig.Slot0.kV = 0.12; // velocity feedforward
      config.fxConfig.Slot0.kA = 0.01;

      // Motion Magic parameters
      config.fxConfig.MotionMagic.MotionMagicCruiseVelocity = 5.0; // rotations per second
      config.fxConfig.MotionMagic.MotionMagicAcceleration = 15.0; // rotations per second^2
      config.fxConfig.MotionMagic.MotionMagicJerk = 100; // limit jerk for smooth motion

      // Gear ratio: motor rotations per turret rotation = GEAR_1/GEAR_0 = 120/60 = 2.0
      config.unitToRotorRatio = 120.0 / 60.0;
      config.momentOfInertia = 0.02; // kg*m^2 for simulation

      config.initialTransform =
          new Transform3d(
              new Translation3d(0, Inches.of(1.75).in(Meters), Inches.of(18.484119).in(Meters)),
              new Rotation3d(0, 0, 0));
    }

    public static int MODEL_INDEX = 3;
    public static int PARENT_INDEX = 0; // drivetrain

    // Gear tooth counts for turret angle calculation
    // Pinion on motor
    public static final double GEAR_0_TOOTH_COUNT = 60.0; // TODO: Replace with actual value
    // attached to e1
    public static final double GEAR_1_TOOTH_COUNT = 120.0; // TODO: Replace with actual value
    // attached to e2
    public static final double GEAR_2_TOOTH_COUNT = 80.0; // TODO: Replace with actual value
    public static final double SLOPE =
        (GEAR_2_TOOTH_COUNT * GEAR_1_TOOTH_COUNT)
            / ((GEAR_1_TOOTH_COUNT - GEAR_2_TOOTH_COUNT) * GEAR_0_TOOTH_COUNT);

    // How many times encoder 1 spins per 1 degree of turret rotation
    public static final double ENCODER_1_TO_TURRET_RATIO = GEAR_1_TOOTH_COUNT / GEAR_0_TOOTH_COUNT;

    // Turret rotation limits
    public static final double FORWARD_LIMIT_DEGREES = 270.0; // TODO: Replace with actual value
    public static final double REVERSE_LIMIT_DEGREES = -270.0; // TODO: Replace with actual value
  }

  public final class Flywheels {
    public static final LoggedTunableMeasure<AngularVelocity> PIDTest =
        new LoggedTunableMeasure<>("Flywheel/PIDTest", RPM.of(2000));

    public static TalonFXSubsystemConfig leaderConfig = new TalonFXSubsystemConfig();
    public static TalonFXSubsystemConfig followerConfig = new TalonFXSubsystemConfig();

    static {
      leaderConfig.name = "Flywheel Leader";
      leaderConfig.talonCANID = new CANDeviceId(2); // Example CAN ID, replace with actual ID
      leaderConfig.fxConfig.Slot0.kP = 0.3;
      leaderConfig.fxConfig.Slot0.kI = 0.0;
      leaderConfig.fxConfig.Slot0.kD = 0.0;
      leaderConfig.fxConfig.Slot0.kS = 0.15;
      leaderConfig.fxConfig.Slot0.kV = 0.114;
      leaderConfig.unitToRotorRatio = 1.0; // 1:1 ratio
      leaderConfig.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      leaderConfig.fxConfig.MotorOutput.PeakReverseDutyCycle = 0;
      leaderConfig.momentOfInertia = 0.01;
      leaderConfig.useFOC = false; // Use VelocityVoltage for sim compatibility
      leaderConfig.tunable = true;

      followerConfig.name = "Flywheel Follower";
      followerConfig.talonCANID = new CANDeviceId(1); // Example CAN ID, replace with actual ID
      followerConfig.unitToRotorRatio = 1.0; // 1:1 ratio
      followerConfig.momentOfInertia = 0.01;
      followerConfig.useFOC = false;
    }

    public static int MODEL_INDEX = 5;
    public static int PARENT_INDEX = 4;

    public static AngularVelocity acceptableError = RPM.of(20);
    public static AngularVelocity idleVelocity = RotationsPerSecond.of(20);

    public static Transform3d localTransform =
        new Transform3d(
            new Translation3d(Inches.of(-5).in(Meters), 0, Inches.of(2).in(Meters)),
            new Rotation3d(0, Degrees.of(-90).in(Radians), 0));

    public static AngularVelocity staticHubVelocity = RotationsPerSecond.of(20);
    public static InterpolatingDoubleTreeMap velocityMap = new InterpolatingDoubleTreeMap();

    public static Distance WHEEL_DIAMETER = Inches.of(4);

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

    static {
      config.name = "Hood";
      config.talonCANID = new CANDeviceId(14); // Example CAN ID, replace with actual ID

      // PID gains for Motion Magic
      config.fxConfig.Slot0.kP = 60.0;
      config.fxConfig.Slot0.kI = 0.0;
      config.fxConfig.Slot0.kD = 5.0;
      config.fxConfig.Slot0.kS = 0.1; // static friction compensation
      config.fxConfig.Slot0.kV = 0.12; // velocity feedforward
      config.fxConfig.Slot0.kA = 0.0;

      // Motion Magic parameters
      config.fxConfig.MotionMagic.MotionMagicCruiseVelocity = 2.0; // rotations per second
      config.fxConfig.MotionMagic.MotionMagicAcceleration = 4.0; // rotations per second^2
      config.fxConfig.MotionMagic.MotionMagicJerk = 0; // no jerk limit

      config.unitToRotorRatio = 1.0; // 1:1 ratio
      config.momentOfInertia = 0.005; // kg*m^2 for simulation

      config.initialTransform =
          new Transform3d(
              new Translation3d(Inches.of(4.086915).in(Meters), 0, 0), new Rotation3d());
    }

    public static Angle retractedPosition = Degrees.of(0);
    public static Angle extendedPosition = Degrees.of(90);
    public static int MODEL_INDEX = 4;
    public static int PARENT_INDEX = 3; // turret

    // Hood rotation limits
    public static final double FORWARD_LIMIT_DEGREES = 60;
    public static final double REVERSE_LIMIT_DEGREES = 0;

    public static Angle staticHubAngle = Degree.of(10);
    public static InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();

    static {
      // Distance (m) -> Hood Pitch (Degrees)
      angleMap.put(0.9, 9.0);
      angleMap.put(1.0, 11.0);
      angleMap.put(1.5, 18.0);
      angleMap.put(2.0, 24.0);
      angleMap.put(3.0, 28.0);
      angleMap.put(4.0, 32.0);
      angleMap.put(4.5, 32.0);
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
