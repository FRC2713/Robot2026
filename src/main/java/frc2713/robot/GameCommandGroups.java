package frc2713.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecondPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc2713.robot.commands.DriveCommands;
import frc2713.robot.subsystems.drive.Drive;
import frc2713.robot.subsystems.intake.IntakeExtension;
import frc2713.robot.subsystems.intake.IntakeRoller;
import frc2713.robot.subsystems.launcher.Flywheels;
import frc2713.robot.subsystems.launcher.Hood;
import frc2713.robot.subsystems.launcher.LauncherConstants;
import frc2713.robot.subsystems.launcher.LaunchingSolutionManager;
import frc2713.robot.subsystems.launcher.Turret;
import frc2713.robot.subsystems.serializer.DyeRotor;
import frc2713.robot.subsystems.serializer.Feeder;
import frc2713.robot.subsystems.serializer.SerializerConstants;
import java.util.Optional;
import java.util.function.DoubleSupplier;

/**
 * A Utility class holding common game actions in the form of command groups that can be shared
 * between Driver, Developer, and AutoRoutines
 */
public final class GameCommandGroups {
  public static final class Launching {

    /** OTF shooting without drive limits. Use for auto routines. */
    public static Command autoOtfShot(
        Drive drive,
        Flywheels flywheels,
        Hood hood,
        Turret turret,
        Feeder feeder,
        DyeRotor dyeRotor,
        IntakeExtension extension,
        IntakeRoller intakeRoller) {
      return Commands.parallel(
              flywheels.otfCommand(),
              hood.otfCommand(),
              turret.otfCommand(),
              intakeRoller.intake(),
              flywheels.simulateLaunchFuelCommand(flywheels::atTarget),
              feeder.feedWhenReady(flywheels::atTarget),
              dyeRotor.feedWhenReady(flywheels::atTarget),
              extension.maintainFuelPressureCommand())
          .withName("OTF Shooting");
    }

    public static Command otfShotHoodProtect(
        Drive drive,
        Flywheels flywheels,
        Hood hood,
        Turret turret,
        Feeder feeder,
        DyeRotor dyeRotor,
        IntakeExtension extension,
        IntakeRoller intakeRoller) {
      return Commands.either(
          Commands.none(),
          otfShot(drive, flywheels, hood, turret, feeder, dyeRotor, extension, intakeRoller),
          () -> hood.inRetractionZone(() -> drive.getPose()));
    }

    /** OTF shooting with drive limits. Use for driver/operator triggers. */
    public static Command otfShot(
        Drive drive,
        Flywheels flywheels,
        Hood hood,
        Turret turret,
        Feeder feeder,
        DyeRotor dyeRotor,
        IntakeExtension extension,
        IntakeRoller intakeRoller) {
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
              intakeRoller.intake(),
              flywheels.simulateLaunchFuelCommand(flywheels::atTarget),
              feeder.feedWhenReady(flywheels::atTarget),
              dyeRotor.dynamicFeedWhenReady(flywheels::atTarget),
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

    public static Command towerShot(
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
          rollers.intake());
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
          rollers.intake());
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
          rollers.intake());
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
              flywheels.simulateLaunchFuelCommand(
                  () -> flywheels.atTarget() && hood.atTarget() && turret.atTarget()),
              feeder.feedWhenReady(
                  () -> flywheels.atTarget() && hood.atTarget() && turret.atTarget()),
              dyeRotor.feedWhenReady(
                  () -> flywheels.atTarget() && hood.atTarget() && turret.atTarget()))
          .withName("Hub Shooting");
    }

    /** Stop shooting and clear drive limits. */
    public static Command stopShooting(
        Drive drive, Feeder feeder, DyeRotor dyeRotor, Flywheels flywheels) {
      return Commands.parallel(
              DriveCommands.clearDriveLimits(drive),
              feeder.stop(),
              dyeRotor.stop(),
              flywheels.stop())
          .withName("Stopped Shooting");
    }

    public static Command stopShootingAndRetractHood(
        Drive drive, Feeder feeder, DyeRotor dyeRotor, Hood hood, Flywheels flywheels) {
      return Commands.parallel(stopShooting(drive, feeder, dyeRotor, flywheels), hood.retract());
    }
  }

  public static final class OperatorOverriderrs {
    public static Command stir(DyeRotor dyeRotor, IntakeRoller rollers) {
      return Commands.parallel(dyeRotor.stirFuel(), rollers.intake());
    }

    public static Command stopStir(DyeRotor dyeRotor, IntakeRoller rollers) {
      return Commands.parallel(dyeRotor.stop(), rollers.stop());
    }

    // this is here just in case we want unjamming to involve more
    public static Command unjam(Feeder feeder) {
      return feeder.setVelocity(SerializerConstants.Feeder.unjammingSpeed);
    }

    // this is here just in case we want unjamming to involve more
    public static Command stopUnjam(Feeder feeder) {
      return Commands.parallel(feeder.stop());
    }

    public static Command outtake(
        IntakeExtension extension, IntakeRoller rollers, DyeRotor dyeRotor) {
      return Commands.parallel(extension.extendCommand(), rollers.outtake(), dyeRotor.stop());
    }
  }

  public static Command intakeAlign(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return DriveCommands.joystickDriveAtAngle(
            RobotContainer.drive,
            xSupplier,
            ySupplier,
            () -> LaunchingSolutionManager.ZoneSelectionHelpers.storedIntakeRotation)
        .withName("Drive Intake Align");
  }
}
