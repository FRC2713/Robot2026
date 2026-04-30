package frc2713.robot.commands.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.Path;
import frc2713.lib.util.WaitSupplierCommand;
import frc2713.robot.GameCommandGroups;
import frc2713.robot.RobotContainer;
import java.util.function.Supplier;

public class BLineMidwarsTrenchified {
  // public static final

  public static Command getCommand(Supplier<Boolean> shouldMirror) {
    return Commands.sequence(
        new WaitSupplierCommand(() -> SmartDashboard.getNumber("autoStartDelay", 0)),
        // Drive through neutral zone with intake and shoot events
        RobotContainer.pathBuilder
            .withPoseReset(RobotContainer.drive::setPose)
            .withShouldMirror(shouldMirror)
            .withEvent(
                "intake",
                GameCommandGroups.Intaking.intake(
                    RobotContainer.intakeExtension, RobotContainer.intakeRoller))
            .withEvent(
                "shoot_1",
                GameCommandGroups.Launching.autoOtfShot(
                    RobotContainer.drive,
                    RobotContainer.flywheels,
                    RobotContainer.hood,
                    RobotContainer.turret,
                    RobotContainer.feeder,
                    RobotContainer.dyeRotor,
                    RobotContainer.intakeExtension,
                    RobotContainer.intakeRoller))
            .build(new Path("mid_wards_trenchified")),
        // Pausing near hub to shoot
        GameCommandGroups.Launching.autoOtfShot(
                RobotContainer.drive,
                RobotContainer.flywheels,
                RobotContainer.hood,
                RobotContainer.turret,
                RobotContainer.feeder,
                RobotContainer.dyeRotor,
                RobotContainer.intakeExtension,
                RobotContainer.intakeRoller)
            .withTimeout(4.5),
        GameCommandGroups.Launching.stopShootingAndRetractHood(
            RobotContainer.drive,
            RobotContainer.feeder,
            RobotContainer.dyeRotor,
            RobotContainer.hood,
            RobotContainer.flywheels).withTimeout(0.25),
        // Drive through trench again
        RobotContainer.pathBuilder
            .withShouldMirror(shouldMirror)
            .withPoseReset(pose -> {})
            .withEvent("intake",
                    GameCommandGroups.Intaking.intake(
                        RobotContainer.intakeExtension, RobotContainer.intakeRoller))
            .withEvent(
                "shoot_2",
                GameCommandGroups.Launching.autoOtfShot(
                        RobotContainer.drive,
                        RobotContainer.flywheels,
                        RobotContainer.hood,
                        RobotContainer.turret,
                        RobotContainer.feeder,
                        RobotContainer.dyeRotor,
                        RobotContainer.intakeExtension,
                        RobotContainer.intakeRoller))
            .build(new Path("mid_wards_trenchified_second")),
        // Drive to trench while shooting
        GameCommandGroups.Launching.autoOtfShot(
            RobotContainer.drive,
            RobotContainer.flywheels,
            RobotContainer.hood,
            RobotContainer.turret,
            RobotContainer.feeder,
            RobotContainer.dyeRotor,
            RobotContainer.intakeExtension,
            RobotContainer.intakeRoller));
  }
}
