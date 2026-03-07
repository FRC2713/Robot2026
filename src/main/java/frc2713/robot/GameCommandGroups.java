package frc2713.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecondPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc2713.robot.commands.DriveCommands;
import frc2713.robot.subsystems.drive.Drive;
import frc2713.robot.subsystems.intake.IntakeConstants;
import frc2713.robot.subsystems.intake.IntakeExtension;
import frc2713.robot.subsystems.intake.IntakeRoller;
import frc2713.robot.subsystems.launcher.Flywheels;
import frc2713.robot.subsystems.launcher.Hood;
import frc2713.robot.subsystems.launcher.LauncherConstants;
import frc2713.robot.subsystems.launcher.Turret;
import frc2713.robot.subsystems.serializer.DyeRotor;
import frc2713.robot.subsystems.serializer.Feeder;
import java.util.Optional;

/**
 * A Utility class holding common game actions in the form of command groups that can be shared
 * between Driver, Developer, and AutoRoutines
 */
public final class GameCommandGroups {
  public static final class Launching {

    /** OTF shooting without drive limits. Use for auto routines. */
    public static Command getOtfShot(
        Flywheels flywheels, Hood hood, Turret turret, Feeder feeder, DyeRotor dyeRotor) {
      return Commands.parallel(
              flywheels.otfCommand(),
              hood.otfCommand(),
              turret.otfCommand(),
              flywheels.simulateLaunchedFuel(flywheels::atTarget),
              feeder.feedWhenReady(flywheels::atTarget),
              dyeRotor.feedWhenReady(flywheels::atTarget))
          .withName("OTF Shooting");
    }

    /** OTF shooting with drive limits. Use for driver/operator triggers. */
    public static Command otfShot(
        Drive drive,
        Flywheels flywheels,
        Hood hood,
        Turret turret,
        Feeder feeder,
        DyeRotor dyeRotor,
        IntakeExtension extension) {
      return Commands.parallel(
              DriveCommands.setDriveLimits(
                  drive,
                  Optional.of(FeetPerSecond.of(4.0)),
                  Optional.of(FeetPerSecondPerSecond.of(12.0)),
                  Optional.of(DegreesPerSecond.of(90.0)),
                  Optional.of(DegreesPerSecondPerSecond.of(360.0))),
              flywheels.otfCommand(),
              hood.otfCommand(),
              turret.otfCommand(),
              flywheels.simulateLaunchedFuel(flywheels::atTarget),
              feeder.feedWhenReady(flywheels::atTarget),
              dyeRotor.feedWhenReady(flywheels::atTarget),
              extension.maintainFuelPressureCommand())
          .withName("OTF Shooting");
    }

    public static Command dumbShot(
        Drive drive,
        Flywheels flywheels,
        Hood hood,
        Turret turret,
        Feeder feeder,
        DyeRotor dyeRotor,
        IntakeExtension extension,
        IntakeRoller rollers) {
      return Commands.parallel(
          flywheels.setVelocity(LauncherConstants.Flywheels.staticTowerVelocity),
          // hood.dumbCommand(),
          // turret.setAngle(() -> Degrees.of(0.0)),
          feeder.feedWhenReady(() -> flywheels.atTarget()),
          dyeRotor.feedWhenReady(() -> flywheels.atTarget())
          // extension.maintainFuelPressureCommand(),
          // rollers.voltageCommand(IntakeConstants.Roller.intakeVoltageDesired)
          );
    }

    public static Command toowerShot(
        Drive drive,
        Flywheels flywheels,
        Hood hood,
        Turret turret,
        Feeder feeder,
        DyeRotor dyeRotor,
        IntakeExtension extension,
        IntakeRoller rollers) {
      return Commands.parallel(
          flywheels.setVelocity(LauncherConstants.Flywheels.staticTowerVelocity),
          hood.setAngleCommand(LauncherConstants.Hood.staticTowerAngle),
          turret.setAngle(LauncherConstants.Turret.staticTowerShot),
          feeder.feedWhenReady(() -> flywheels.atTarget()),
          dyeRotor.feedWhenReady(() -> flywheels.atTarget()),
          extension.maintainFuelPressureCommand(),
          rollers.voltageCommand(IntakeConstants.Roller.intakeVoltageDesired));
    }

    public static Command leftTrenchShot(
        Drive drive,
        Flywheels flywheels,
        Hood hood,
        Turret turret,
        Feeder feeder,
        DyeRotor dyeRotor,
        IntakeExtension extension,
        IntakeRoller rollers) {
      return Commands.parallel(
          flywheels.setVelocity(LauncherConstants.Flywheels.staticRightLeftTrench),
          hood.setAngleCommand(LauncherConstants.Hood.staticRightLeftTrenchAngle),
          turret.setAngle(LauncherConstants.Turret.staticLeftTrench),
          feeder.feedWhenReady(() -> flywheels.atTarget()),
          dyeRotor.feedWhenReady(() -> flywheels.atTarget()),
          extension.maintainFuelPressureCommand(),
          rollers.voltageCommand(IntakeConstants.Roller.intakeVoltageDesired));
    }

    public static Command rightTrenchShot(
        Drive drive,
        Flywheels flywheels,
        Hood hood,
        Turret turret,
        Feeder feeder,
        DyeRotor dyeRotor,
        IntakeExtension extension,
        IntakeRoller rollers) {
      return Commands.parallel(
          flywheels.setVelocity(LauncherConstants.Flywheels.staticRightLeftTrench),
          hood.setAngleCommand(LauncherConstants.Hood.staticRightLeftTrenchAngle),
          turret.setAngle(LauncherConstants.Turret.staticRightTrench),
          feeder.feedWhenReady(() -> flywheels.atTarget()),
          dyeRotor.feedWhenReady(() -> flywheels.atTarget()),
          extension.maintainFuelPressureCommand(),
          rollers.voltageCommand(IntakeConstants.Roller.intakeVoltageDesired));
    }

    /** Hub shooting command. */
    public static Command hubShot(
        Drive drive,
        Flywheels flywheels,
        Hood hood,
        Turret turret,
        Feeder feeder,
        DyeRotor dyeRotor) {
      return Commands.parallel(
              flywheels.hubCommand(),
              hood.hubCommand(),
              turret.hubCommand(drive::getPose),
              flywheels.simulateLaunchedFuel(
                  () -> flywheels.atTarget() && hood.atTarget() && turret.atTarget()),
              feeder.feedWhenReady(
                  () -> flywheels.atTarget() && hood.atTarget() && turret.atTarget()),
              dyeRotor.feedWhenReady(
                  () -> flywheels.atTarget() && hood.atTarget() && turret.atTarget()))
          .withName("Hub Shooting");
    }

    /** Stop shooting and clear drive limits. */
    public static Command stopShooting(Drive drive, Feeder feeder, DyeRotor dyeRotor) {
      return Commands.parallel(
              DriveCommands.clearDriveLimits(drive), feeder.stop(), dyeRotor.stopCommand())
          .withName("Stopped Shooting");
    }

    public static Command stopShootingAndRetractHub(
        Drive drive, Feeder feeder, DyeRotor dyeRotor, Hood hood) {
      return Commands.parallel(stopShooting(drive, feeder, dyeRotor), hood.retract());
    }
  }
}
