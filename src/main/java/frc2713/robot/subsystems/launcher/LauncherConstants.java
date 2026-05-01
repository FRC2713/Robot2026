package frc2713.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Hertz;
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
    public static Angle manualOffset = Degrees.of(180);

    // Turret rotation limits
    public static final Angle forwardSoftLimit = Degrees.of(180);
    public static final Angle reverseSoftLimit = Degrees.of(-180);

    public static final LoggedTunableMeasure<Time> OTF_OMEGA_LOOKAHEAD =
        new LoggedTunableMeasure<Time>("LaunchingSolutionManager/omegaTime", Seconds.of(0.05));

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
      config.generalControlMode = GeneralControlMode.POSITION;
      config.acceptablePositionError = Degrees.of(3);

      config.fxConfig.Feedback.FeedbackRotorOffset = 0.;

      config.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      config.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

      // PID gains for Motion Magic
      config.fxConfig.Slot0.kP = Util.modeDependentValue(80.0, 80.0);
      config.fxConfig.Slot0.kI = 0.0;
      config.fxConfig.Slot0.kD = Util.modeDependentValue(0.7, 0.7);
      config.fxConfig.Slot0.kS = Util.modeDependentValue(0.3, 0.3); // static friction compensation
      config.fxConfig.Slot0.kV = Util.modeDependentValue(2.9, 2.9); // velocity feedforward
      config.fxConfig.Slot0.kA = Util.modeDependentValue(0.0, 0.0); // acceleration feedforward

      // Motion Magic parameters
      config.fxConfig.MotionMagic.MotionMagicCruiseVelocity = 20.0 / 8; // rotations per second
      config.fxConfig.MotionMagic.MotionMagicAcceleration = 50.0 / 4; // rotations per second^2
      config.fxConfig.MotionMagic.MotionMagicJerk = 0.; // limit jerk for smooth motion

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
              new Rotation3d(0, 0, manualOffset.in(Radians)));

      config.fxConfig.CurrentLimits.StatorCurrentLimit = 100;
      config.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;

      config.fxConfig.CurrentLimits.SupplyCurrentLimit = 45;
      config.fxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
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
        new LoggedTunableMeasure<>("Turret/Turret Static Trench L", Degrees.of(-190));
    public static LoggedTunableMeasure<Angle> staticRightTrench =
        new LoggedTunableMeasure<>("Turret/Turret Static Trench R", Degrees.of(190));
    public static LoggedTunableMeasure<Angle> staticTowerShot =
        new LoggedTunableMeasure<>("Turret/Turret Static Tower", Degrees.of(0.0));
  }

  public final class Flywheels {

    public static TalonFXSubsystemConfig leaderConfig = new TalonFXSubsystemConfig();
    public static TalonFXSubsystemConfig followerConfig = new TalonFXSubsystemConfig();
    public static MomentOfInertia flywhMomentOfInertia = MoiUnits.PoundSquareInches.of(10.410164);
    public static double gearRatio = 24.0 / 18.0; // 1.33:1 reduction from motor to flywheel

    static {
      leaderConfig.name = "Flywheels";
      leaderConfig.talonCANID = new CANDeviceId(50, "canivore");
      leaderConfig.fxConfig.Slot0.kP = Util.modeDependentValue(.7, 1.0); // last foc val: 400.0
      leaderConfig.fxConfig.Slot0.kI = 0.0;
      leaderConfig.fxConfig.Slot0.kD = 0.004;
      leaderConfig.fxConfig.Slot0.kS = Util.modeDependentValue(0.2, 0.0); // last foc val: 2.0
      leaderConfig.fxConfig.Slot0.kV = 0.12 * gearRatio; // last foc val: 0
      leaderConfig.fxConfig.CurrentLimits.StatorCurrentLimit = 180.0;
      leaderConfig.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      leaderConfig.fxConfig.CurrentLimits.SupplyCurrentLimit = 80.0;
      leaderConfig.fxConfig.CurrentLimits.SupplyCurrentLowerTime = 0.0;
      leaderConfig.fxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      leaderConfig.simOrientation = ChassisReference.CounterClockwise_Positive;
      leaderConfig.unitToRotorRatio = gearRatio;
      leaderConfig.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      leaderConfig.fxConfig.Voltage.PeakReverseVoltage = 0;
      leaderConfig.fxConfig.TorqueCurrent.PeakReverseTorqueCurrent = 0;
      leaderConfig.momentOfInertia = flywhMomentOfInertia.times(0.5);
      leaderConfig.useFOC = false;
      leaderConfig.fxConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      leaderConfig.generalControlMode = GeneralControlMode.VELOCITY;
      leaderConfig.acceptableVelocityError = RPM.of(100);
      leaderConfig.velocityControlFrequency = Hertz.of(100); // CAN FD can go up to 1000

      followerConfig.name = "Flywheels Follower";
      followerConfig.talonCANID = new CANDeviceId(51, "canivore");
      followerConfig.unitToRotorRatio = gearRatio;
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

    public static Distance WHEEL_DIAMETER = Inches.of(4);
    // How many fuel we can launch per second at max firing rate
    public static double launchRateFuelPerSecond = 9.0;

    // the volume of fuel we're launching per second at max firing rate
    public static double launchRateVolumeInchesCubedPerSecond =
        launchRateFuelPerSecond * GamePieceConstants.Fuel.volumeInchesCubed;

    public static LoggedTunableMeasure<AngularVelocity> idleVelocity =
        new LoggedTunableMeasure<>("Flywheels/Idle Velocity", RPM.of(300));
    public static LoggedTunableMeasure<AngularVelocity> PIDTest =
        new LoggedTunableMeasure<>("Flywheels/PIDTest", RPM.of(4000));
    public static LoggedTunableMeasure<AngularVelocity> staticRightLeftTrench =
        new LoggedTunableMeasure<>("Flywheels/Flywheels Static Trench", RPM.of(2850));
    public static LoggedTunableMeasure<AngularVelocity> staticHubVelocity =
        new LoggedTunableMeasure<>("Flywheels/Flywheels Static Hub", RotationsPerSecond.of(25));
    public static LoggedTunableMeasure<AngularVelocity> staticTowerVelocity =
        new LoggedTunableMeasure<AngularVelocity>("Flywheels/Flywheels Static Tower", RPM.of(2400));
  }

  public final class Hood {

    public static TalonFXSubsystemConfig config = new TalonFXSubsystemConfig();

    public static final Angle minAngle = Degrees.of(0);
    public static final Angle maxAngle = Degrees.of(30);

    // 8t pinion to 20t gear, 13t gear to  30t gear, 10t gear to 146 sector gear
    public static double gearRatio = ((20 / 8.0) * (30.0 / 13.0) * (146.0 / 10.0));

    public static final Angle retractedPosition = Degrees.of(0.5);

    static {
      config.name = "Hood";
      config.talonCANID = new CANDeviceId(54, "canivore"); // Example CAN ID, replace with actual ID

      config.fxConfig.Feedback.FeedbackRotorOffset = 0.067871;

      // PID gains for Motion Magic
      config.fxConfig.Slot0.kP = 400.0;
      config.fxConfig.Slot0.kI = 0.0;
      config.fxConfig.Slot0.kD = 4.0;
      config.fxConfig.Slot0.kS = 1.0; // static friction compensation
      config.fxConfig.Slot0.kV = 0.092 * gearRatio; // velocity feedforward
      config.fxConfig.Slot0.kA = 0.0;

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

    public static LoggedTunableMeasure<Angle> staticTowerAngle =
        new LoggedTunableMeasure<Angle>("Hood/Hood Static Tower", Degrees.of(20));
    public static LoggedTunableMeasure<Angle> staticRightLeftTrenchAngle =
        new LoggedTunableMeasure<Angle>("Hood/Hood Static Trench", Degrees.of(25));
    public static LoggedTunableMeasure<Angle> staticHubAngle =
        new LoggedTunableMeasure<Angle>("Hood/Hood Static Hub", Degrees.of(25));
  }

  /**
   * How {@link frc2713.robot.subsystems.launcher.LaunchingSolutionManager} picks a firing solution.
   * {@link LaunchSolverMode#VECTOR_APPROX} and {@link LaunchSolverMode#ITOF} are specific
   * strategies.
   */
  public enum LaunchSolverMode {
    /** {@code calculateStatic} */
    STATIC,
    /**
     * Match the static ideal ground-frame launch vector by solving for muzzle RPM/hood from the
     * resultant velocity after subtracting chassis motion (first-order vector approximation).
     */
    VECTOR_APPROX,
    /** {@code calculateITOF} — iterative time-of-flight while moving */
    ITOF;
  }

  public static LoggedTunableBoolean otfFutureProjectionEnabled =
      new LoggedTunableBoolean("LaunchingSolutionManager/projection_enabled", true);
}
