package frc2713.robot.oi;

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
        .leftBumper()
        .onTrue(
            GameCommandGroups.Launching.leftTrenchShot(
                    drive, flywheels, hood, turret, feeder, dyeRotor, intakeExtension, intakeRoller)
                .withName("Static LTrench Shot"))
        .onFalse(
            GameCommandGroups.Launching.stopShootingAndRetractHood(
                drive, feeder, dyeRotor, hood, flywheels));

    controller
        .rightBumper()
        .onTrue(
            GameCommandGroups.Launching.rightTrenchShot(
                drive, flywheels, hood, turret, feeder, dyeRotor, intakeExtension, intakeRoller))
        .onFalse(
            GameCommandGroups.Launching.stopShootingAndRetractHood(
                    drive, feeder, dyeRotor, hood, flywheels)
                .withName("Static R Trench Shot"));

    controller
        .rightTrigger(0.98)
        .onTrue(GameCommandGroups.OperatorOverriderrs.unjam(feeder).withName("Unjamming"))
        .onFalse(
            GameCommandGroups.OperatorOverriderrs.stopUnjam(feeder).withName("Stop Unjamming"));

    controller
        .leftTrigger(0.98)
        .onTrue(GameCommandGroups.OperatorOverriderrs.stir(dyeRotor, intakeRoller).withName("Stir"))
        .onFalse(
            GameCommandGroups.OperatorOverriderrs.stopStir(dyeRotor, intakeRoller)
                .withName("Stop Stir"));

    controller
        .y()
        .onTrue(
            GameCommandGroups.Launching.towerShot(
                drive, flywheels, hood, turret, feeder, dyeRotor, intakeExtension, intakeRoller))
        .onFalse(
            GameCommandGroups.Launching.stopShootingAndRetractHood(
                drive, feeder, dyeRotor, hood, flywheels));

    // disable ducking
    controller.a().onTrue(Commands.run(() -> hood.disableDucking = !hood.disableDucking));
  }
}
