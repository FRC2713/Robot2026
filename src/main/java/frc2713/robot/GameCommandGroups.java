package frc2713.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc2713.robot.commands.DriveCommands;
import frc2713.robot.subsystems.drive.Drive;
import frc2713.robot.subsystems.drive.DriveConstants;
import frc2713.robot.subsystems.intake.IntakeExtension;
import frc2713.robot.subsystems.intake.IntakeExtension.FuelPressureType;
import frc2713.robot.subsystems.intake.IntakeRoller;
import frc2713.robot.subsystems.launcher.Flywheels;
import frc2713.robot.subsystems.launcher.Hood;
import frc2713.robot.subsystems.launcher.LauncherConstants;
import frc2713.robot.subsystems.launcher.LaunchingSolutionManager;
import frc2713.robot.subsystems.launcher.LaunchingSolutionManager.LaunchingAction;
import frc2713.robot.subsystems.launcher.Turret;
import frc2713.robot.subsystems.serializer.DyeRotor;
import frc2713.robot.subsystems.serializer.Feeder;
import frc2713.robot.subsystems.serializer.SerializerConstants;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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
        IntakeRoller intakeRoller,
        Supplier<Double> fuelPressureDelay) {
      return Commands.either(
              Commands.print("[AUTO] Auto in neutral zone!"),
              Commands.parallel(
                  flywheels.otfCommand(),
                  hood.otfCommand(),
                  turret.otfCommand(),
                  intakeRoller.intake(),
                  flywheels.simulateLaunchFuelCommand(
                      () -> flywheels.atTarget() && hood.atTarget()),
                  feeder.feedWhenReady(
                      () -> flywheels.atTarget() && hood.atTarget(), Seconds.of(0.8)),
                  dyeRotor.feedWhenReady(
                      () -> flywheels.atTarget() && hood.atTarget(), Seconds.of(0.8)),
                  extension.maintainFuelPressureCommand(
                      FuelPressureType.OSCILLATING,
                      fuelPressureDelay.get())), // retract pressure type had 1.0 delay
              () -> FieldConstants.NeutralZone.region.contains(drive.getPose().getTranslation()))
          .withName("Auto OTF Shooting");
    }

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
      return autoOtfShot(
          drive, flywheels, hood, turret, feeder, dyeRotor, extension, intakeRoller, () -> 0.5);
    }

    /**
     * OTF shooting without drive limits and a pressure delay longer than auto. Use for auto
     * routines.
     */
    public static Command autoOtfShotNoPressure(
        Drive drive,
        Flywheels flywheels,
        Hood hood,
        Turret turret,
        Feeder feeder,
        DyeRotor dyeRotor,
        IntakeExtension extension,
        IntakeRoller intakeRoller) {
      return autoOtfShot(
          drive, flywheels, hood, turret, feeder, dyeRotor, extension, intakeRoller, () -> 30.0);
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
              () -> hood.inRetractionZone(() -> drive.getPose()))
          .withName("OTF Shooting w Hood Protect");
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
              Commands.either(
                  DriveCommands.setDriveLimits(drive, DriveConstants.scoringDriveLimits),
                  DriveCommands.setDriveLimits(drive, DriveConstants.feedingDriveLimits),
                  () -> LaunchingSolutionManager.currentAction == LaunchingAction.SCORING),
              flywheels.otfCommand(),
              hood.otfCommand(),
              turret.otfCommand(),
              flywheels.simulateLaunchFuelCommand(flywheels::atTarget),
              Commands.sequence(
                  feeder.voltageCommand(() -> Volts.of(-12)).withTimeout(0.25),
                  feeder.feedWhenReady(flywheels::atTarget)),
              dyeRotor.dynamicFeedWhenReady(flywheels::atTarget, intakeRoller::isIntaking))
          .withName("OTF Shooting");
    }
    /** OTF shooting with drive limits. Use for driver/operator triggers. */
    public static Command otfShotWithDriveTrain(
        Drive drive,
        Flywheels flywheels,
        Hood hood,
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
              flywheels.simulateLaunchFuelCommand(flywheels::atTarget),
              feeder.feedWhenReady(flywheels::atTarget),
              dyeRotor.dynamicFeedWhenReady(flywheels::atTarget, intakeRoller::isIntaking))
          .withName("OTF Shooting");
    }

    /**
     * OTF shooting with drive limits. Use for driver/operator triggers. If the turret has been
     * broken and the turret can not move.
     */
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
          dyeRotor.feedWhenReady(() -> flywheels.atTarget()));
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
          dyeRotor.feedWhenReady(() -> flywheels.atTarget()));
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
          dyeRotor.feedWhenReady(() -> flywheels.atTarget()));
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

  public static final class Intaking {
    public static Command intake(IntakeExtension extension, IntakeRoller rollers) {
      return Commands.parallel(extension.extendCommand(), rollers.intake()).withName("Intake");
    }

    public static Command outtake(IntakeExtension extension, IntakeRoller rollers) {
      return Commands.parallel(extension.extendCommand(), rollers.outtake()).withName("Outtake");
    }

    public static Command stopIntake(IntakeExtension extension, IntakeRoller rollers) {
      return Commands.parallel(rollers.stop()).withName("Stop Intaking");
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

  public static Command staticTurretOtf(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return DriveCommands.joystickDriveAtAngle(
            RobotContainer.drive, xSupplier, ySupplier, () -> Drive.storedStaticShotRotation)
        .withName("Drive Intake Align");
  }
}
