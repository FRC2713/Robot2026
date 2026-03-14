package frc2713.robot.oi;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Commands;
import frc2713.robot.GameCommandGroups;
import frc2713.robot.subsystems.drive.Drive;
import frc2713.robot.subsystems.intake.IntakeExtension;
import frc2713.robot.subsystems.intake.IntakeRoller;
import frc2713.robot.subsystems.launcher.Flywheels;
import frc2713.robot.subsystems.launcher.Hood;
import frc2713.robot.subsystems.launcher.Turret;
import frc2713.robot.subsystems.serializer.DyeRotor;
import frc2713.robot.subsystems.serializer.Feeder;

public class OperatorControls {
  private final CommandVader4Controller controller = new CommandVader4Controller(1);

  private final Drive drive;
  private final Flywheels flywheels;
  private final Turret turret;
  private final Hood hood;
  private final IntakeRoller intakeRoller;
  private final IntakeExtension intakeExtension;
  private final DyeRotor dyeRotor;
  private final Feeder feeder;

  public OperatorControls(
      Drive drive,
      Flywheels flywheels,
      Turret turret,
      Hood hood,
      IntakeRoller intakeRollers,
      IntakeExtension intakeExtension,
      DyeRotor dyeRotor,
      Feeder feeder) {
    this.drive = drive;
    this.flywheels = flywheels;
    this.turret = turret;
    this.hood = hood;
    this.intakeRoller = intakeRollers;
    this.intakeExtension = intakeExtension;
    this.dyeRotor = dyeRotor;
    this.feeder = feeder;
  }

  public void configureButtonBindings() {

    controller
        .povUp()
        .onTrue(
            Commands.runOnce(() -> flywheels.fudgeFactor = flywheels.fudgeFactor.plus(RPM.of(100)))
                .withName("flywheels fudgeFactor up"));

    controller
        .povDown()
        .onTrue(
            Commands.runOnce(() -> flywheels.fudgeFactor = flywheels.fudgeFactor.minus(RPM.of(100)))
                .withName("flywheels fudgeFactor down"));
    controller
        .povRight()
        .onTrue(
            Commands.runOnce(() -> turret.fudgeFactor = turret.fudgeFactor.minus(Degrees.of(5)))
                .withName("turret fudgeFactor minus"));

    controller
        .povLeft()
        .onTrue(
            Commands.runOnce(() -> turret.fudgeFactor = turret.fudgeFactor.plus(Degrees.of(5)))
                .withName("turret fudgeFactor plus"));

    controller
        .start()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      turret.fudgeFactor = Degrees.of(0.0);
                      flywheels.fudgeFactor = RPM.of(0.0);
                    })
                .withName("fudgeFacor reset"));

    controller
        .back()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      turret.fudgeFactor = Degrees.of(0.0);
                      flywheels.fudgeFactor = RPM.of(0.0);
                    })
                .withName("fudgeFacor reset"));

    controller
        .leftBumper()
        .onTrue(
            GameCommandGroups.Launching.leftTrenchShot(
                    drive, flywheels, hood, turret, feeder, dyeRotor, intakeExtension, intakeRoller)
                .withName("Static LTrench Shot"))
        .onFalse(
            GameCommandGroups.Launching.stopShootingAndRetractHood(
                    drive, feeder, dyeRotor, hood, flywheels)
                .withName("Stop Shooting + Hood Retract"));

    controller
        .rightBumper()
        .onTrue(
            GameCommandGroups.Launching.rightTrenchShot(
                    drive, flywheels, hood, turret, feeder, dyeRotor, intakeExtension, intakeRoller)
                .withName("Static RTrench Shot"))
        .onFalse(
            GameCommandGroups.Launching.stopShootingAndRetractHood(
                    drive, feeder, dyeRotor, hood, flywheels)
                .withName("Stop Shooting + Hood Retract"));

    controller
        .rightTrigger(0.98)
        .onTrue(GameCommandGroups.OperatorOverriderrs.unjam(feeder).withName("Unjamming"))
        .onFalse(
            GameCommandGroups.OperatorOverriderrs.stopUnjam(feeder).withName("Stop Unjamming"));

    controller
        .leftTrigger(0.98)
        .onTrue(dyeRotor.stirFuel().withName("Stir"))
        .onFalse(dyeRotor.stop().withName("Stop Stir"));

    controller
        .b()
        .whileTrue(
            GameCommandGroups.OperatorOverriderrs.outtake(intakeExtension, intakeRoller, dyeRotor)
                .withName("Outtake"));

    controller
        .y()
        .onTrue(
            GameCommandGroups.Launching.towerShot(
                    drive, flywheels, hood, turret, feeder, dyeRotor, intakeExtension, intakeRoller)
                .withName("Static Tower Shot"))
        .onFalse(
            GameCommandGroups.Launching.stopShootingAndRetractHood(
                    drive, feeder, dyeRotor, hood, flywheels)
                .withName("Stop Shooting + Hood Retract"));

    // disable ducking
    controller.a().onTrue(Commands.run(() -> hood.disableDucking = !hood.disableDucking));
  }
}
