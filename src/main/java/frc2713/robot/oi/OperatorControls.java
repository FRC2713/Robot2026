package frc2713.robot.oi;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc2713.robot.GameCommandGroups;
import frc2713.robot.RobotContainer;
import frc2713.robot.subsystems.drive.Drive;
import frc2713.robot.subsystems.intake.IntakeExtension;
import frc2713.robot.subsystems.intake.IntakeRoller;
import frc2713.robot.subsystems.launcher.Flywheels;
import frc2713.robot.subsystems.launcher.Hood;
import frc2713.robot.subsystems.launcher.Turret;
import frc2713.robot.subsystems.serializer.DyeRotor;
import frc2713.robot.subsystems.serializer.Feeder;
import frc2713.robot.util.ShiftManager;

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
            Commands.runOnce(() -> hood.fudgeFactor = hood.fudgeFactor.plus(Degrees.of(1)))
                .withName("hood fudgeFactor up"));

    controller
        .povDown()
        .onTrue(
            Commands.runOnce(() -> hood.fudgeFactor = hood.fudgeFactor.minus(Degrees.of(1)))
                .withName("hood fudgeFactor down"));

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
            Commands.sequence(
                    Commands.runOnce(
                        () -> {
                          turret.fudgeFactor = Degrees.of(0.0);
                          flywheels.fudgeFactor = RPM.of(0.0);
                          hood.fudgeFactor = Degrees.of(0.0);
                        }),
                    Turret.changeDefaultTurretCommand(
                        turret, turret.otfCommand(), "Reset Turret Command"))
                .withName("fudgeFacor reset"));

    controller
        .back()
        .onTrue(
            Commands.sequence(
                    Commands.runOnce(
                        () -> {
                          turret.fudgeFactor = Degrees.of(0.0);
                          flywheels.fudgeFactor = RPM.of(0.0);
                          hood.fudgeFactor = Degrees.of(0.0);
                        }),
                    Turret.changeDefaultTurretCommand(
                        turret, turret.otfCommand(), "Reset Turret Command"))
                .withName("fudgeFacor reset"));

    controller
        .leftBumper()
        .onTrue(intakeExtension.retractCommand())
        .onFalse(intakeExtension.extendCommand());

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
    // controller.a().onTrue(Commands.run(() -> hood.disableDucking = !hood.disableDucking));
    controller
        .a()
        .onTrue(
            Turret.changeDefaultTurretCommand(turret, turret.manualControl(), "Disable Turret"));

    controller
        .x()
        .whileTrue(
            Commands.run(
                () -> {
                  System.out.println("HARD RESET VISION!");
                  var visionPose = RobotContainer.vision.getPose();
                  if (visionPose.isPresent()) {
                    RobotContainer.drive.setPose(visionPose.get());
                  }
                }));

    new Trigger(() -> ShiftManager.getTimeLeftInShift(DriverStation.getMatchTime()) <= 2)
        .whileTrue(controller.RumbleForDuration(0.5));
  }
}
