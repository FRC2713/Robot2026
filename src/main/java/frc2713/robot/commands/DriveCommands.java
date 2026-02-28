// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc2713.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc2713.robot.subsystems.drive.Drive;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double DEADBAND = 0.02;
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
  public static final DoubleSupplier INCH_SPEED = () -> 0.1;

  private DriveCommands() {}

  public static void setDefaultDriveCommand(Drive drive, Command cmd, String name) {
    Logger.recordOutput("CurrentDriveCommand", name);
    var currentCmd = drive.getCurrentCommand();
    if (currentCmd != null) {
      drive.getCurrentCommand().cancel();
      drive.removeDefaultCommand();
    }
    drive.setDefaultCommand(cmd);
  }

  public static Command changeDefaultDriveCommand(Drive drive, Command cmd, String name) {
    return Commands.runOnce(() -> setDefaultDriveCommand(drive, cmd, name));
  }

  public static Command inch(Drive drive, DoubleSupplier xSupplier) {
    return Commands.run(
        () -> {
          drive.runVelocity(new ChassisSpeeds(0, xSupplier.getAsDouble(), 0));
        },
        drive);
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  // ----- Drive Limit Commands -----
  // These commands let you set or clear the drive limits from button bindings, auto routines, etc.

  /** Returns a command that caps the robot's maximum driving speed. */
  public static Command setLinearVelocityLimit(Drive drive, LinearVelocity maxLinearVelocity) {
    return Commands.runOnce(() -> drive.setLinearVelocityLimit(maxLinearVelocity));
  }

  /** Returns a command that caps how fast the robot can change its driving speed (m/s^2). */
  public static Command setLinearAccelerationLimit(
      Drive drive, LinearAcceleration maxLinearAcceleration) {
    return Commands.runOnce(() -> drive.setLinearAccelerationLimit(maxLinearAcceleration));
  }

  /** Returns a command that caps the robot's maximum turning speed (rad/s). */
  public static Command setAngularVelocityLimit(Drive drive, AngularVelocity maxAngularVelocity) {
    return Commands.runOnce(() -> drive.setAngularVelocityLimit(maxAngularVelocity));
  }

  /** Returns a command that caps how fast the robot can change its turning speed (rad/s^2). */
  public static Command setAngularAccelerationLimit(
      Drive drive, AngularAcceleration maxAngularAcceleration) {
    return Commands.runOnce(() -> drive.setAngularAccelerationLimit(maxAngularAcceleration));
  }

  /** Returns a command that removes all drive limits, restoring full speed. */
  public static Command clearDriveLimits(Drive drive) {
    return Commands.runOnce(drive::clearDriveLimits);
  }

  /**
   * Returns a command that sets whichever drive limits you provide. Any limit passed as
   * Optional.empty() will be left unchanged from whatever it was before.
   *
   * <p>Example: only limit linear velocity to 2 m/s, leave everything else alone:
   *
   * <pre>
   *   DriveCommands.setDriveLimits(drive,
   *       Optional.of(MetersPerSecond.of(2.0)),
   *       Optional.empty(),
   *       Optional.empty(),
   *       Optional.empty())
   * </pre>
   */
  public static Command setDriveLimits(
      Drive drive,
      Optional<LinearVelocity> linearVelocity,
      Optional<LinearAcceleration> linearAcceleration,
      Optional<AngularVelocity> angularVelocity,
      Optional<AngularAcceleration> angularAcceleration) {
    return Commands.runOnce(
        () -> {
          // Only apply limits that were provided; skip the rest so they stay as-is
          linearVelocity.ifPresent(drive::setLinearVelocityLimit);
          linearAcceleration.ifPresent(drive::setLinearAccelerationLimit);
          angularVelocity.ifPresent(drive::setAngularVelocityLimit);
          angularAcceleration.ifPresent(drive::setAngularAccelerationLimit);
        });
  }

  /**
   * Wraps another command so that drive limits are active while it runs. When the inner command
   * starts, the limits are applied. When it ends (for any reason), the limits are automatically
   * cleared.
   *
   * <p>Example usage:
   *
   * <pre>
   *   // Limit to 2 m/s and 3 m/s^2 linear while scoring, no angular limits
   *   DriveCommands.withDriveLimits(drive,
   *       MetersPerSecond.of(2.0),
   *       MetersPerSecondPerSecond.of(3.0),
   *       RadiansPerSecond.of(Double.POSITIVE_INFINITY),
   *       RadiansPerSecondPerSecond.of(Double.POSITIVE_INFINITY),
   *       scoringCommand)
   * </pre>
   *
   * @param drive the drive subsystem
   * @param linearVelLimit max linear velocity
   * @param linearAccelLimit max linear acceleration
   * @param angularVelLimit max angular velocity
   * @param angularAccelLimit max angular acceleration
   * @param inner the command to run while limits are active
   */
  public static Command withDriveLimits(
      Drive drive,
      LinearVelocity linearVelLimit,
      LinearAcceleration linearAccelLimit,
      AngularVelocity angularVelLimit,
      AngularAcceleration angularAccelLimit,
      Command inner) {
    return inner
        .beforeStarting(
            () -> {
              drive.setLinearVelocityLimit(linearVelLimit);
              drive.setLinearAccelerationLimit(linearAccelLimit);
              drive.setAngularVelocityLimit(angularVelLimit);
              drive.setAngularAccelerationLimit(angularAccelLimit);
            })
        .finallyDo(() -> drive.clearDriveLimits());
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }
}
