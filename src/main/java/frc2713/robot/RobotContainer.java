// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc2713.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc2713.lib.io.MotorIO;
import frc2713.lib.io.SimTalonFXIO;
import frc2713.lib.io.TalonFXIO;
import frc2713.lib.subsystem.KinematicsManager;
import frc2713.robot.commands.DriveCommands;
import frc2713.robot.generated.TunerConstants;
import frc2713.robot.oi.DriverControls;
import frc2713.robot.subsystems.climber.Climber;
import frc2713.robot.subsystems.climber.ClimberConstants;
import frc2713.robot.subsystems.drive.Drive;
import frc2713.robot.subsystems.drive.GyroIO;
import frc2713.robot.subsystems.drive.ModuleIO;
import frc2713.robot.subsystems.drive.ModuleIOSim;
import frc2713.robot.subsystems.intake.IntakeConstants;
import frc2713.robot.subsystems.intake.IntakeExtension;
import frc2713.robot.subsystems.intake.IntakeRoller;
import frc2713.robot.subsystems.launcher.Flywheels;
import frc2713.robot.subsystems.launcher.Hood;
import frc2713.robot.subsystems.launcher.LauncherConstants;
import frc2713.robot.subsystems.launcher.LaunchingSolutionManager;
import frc2713.robot.subsystems.launcher.Turret;
import frc2713.robot.subsystems.launcher.TurretMotorIO;
import frc2713.robot.subsystems.launcher.TurretMotorIOSim;
import frc2713.robot.subsystems.launcher.TurretMotorIOTalonFX;
import frc2713.robot.subsystems.launcher.TurretSubsystemConfig;
import frc2713.robot.subsystems.serializer.DyeRotor;
import frc2713.robot.subsystems.serializer.Feeder;
import frc2713.robot.subsystems.serializer.SerializerConstants;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final KinematicsManager kinematicsManager = new KinematicsManager();
  private final LaunchingSolutionManager launchingSolutionManager = new LaunchingSolutionManager();
  // Subsystems
  public static Drive drive;
  private final Flywheels flywheels;
  private final Turret turret;
  private final Hood hood;
  private final IntakeRoller intakeRoller;
  private final IntakeExtension intakeExtension;
  private final DyeRotor dyeRotor;
  private final Feeder feeder;
  private final Climber climber;
  // Controllers
  public static DriverControls driverControls = new DriverControls();

  // Controller

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        flywheels =
            new Flywheels(
                LauncherConstants.Flywheels.leftConfig,
                LauncherConstants.Flywheels.rightConfig,
                new TalonFXIO(LauncherConstants.Flywheels.leftConfig),
                new TalonFXIO(LauncherConstants.Flywheels.rightConfig));

        hood = new Hood(LauncherConstants.Hood.config, new MotorIO() {});

        turret =
            new Turret(
                LauncherConstants.Turret.config,
                new TurretMotorIOTalonFX(LauncherConstants.Turret.config));
                
        intakeRoller =
            new IntakeRoller(
                IntakeConstants.Roller.config, new TalonFXIO(IntakeConstants.Roller.config));
        intakeExtension =
            new IntakeExtension(
                IntakeConstants.Extension.config, new TalonFXIO(IntakeConstants.Extension.config));
        dyeRotor =
            new DyeRotor(
                SerializerConstants.DyeRotor.config,
                new TalonFXIO(SerializerConstants.DyeRotor.config));
        feeder =
            new Feeder(
                SerializerConstants.Feeder.config,
                new TalonFXIO(SerializerConstants.Feeder.config));
        climber = new Climber(ClimberConstants.config, new TalonFXIO(ClimberConstants.config));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        flywheels =
            new Flywheels(
                LauncherConstants.Flywheels.leftConfig,
                LauncherConstants.Flywheels.rightConfig,
                new SimTalonFXIO(LauncherConstants.Flywheels.leftConfig),
                new SimTalonFXIO(LauncherConstants.Flywheels.rightConfig));
        hood =
            new Hood(
                LauncherConstants.Hood.config, new SimTalonFXIO(LauncherConstants.Hood.config));

        turret =
            new Turret(
                LauncherConstants.Turret.config,
                new TurretMotorIOSim(LauncherConstants.Turret.config));
        intakeRoller =
            new IntakeRoller(
                IntakeConstants.Roller.config, new SimTalonFXIO(IntakeConstants.Roller.config));
        intakeExtension =
            new IntakeExtension(
                IntakeConstants.Extension.config,
                new SimTalonFXIO(IntakeConstants.Extension.config));
        dyeRotor =
            new DyeRotor(
                SerializerConstants.DyeRotor.config,
                new SimTalonFXIO(SerializerConstants.DyeRotor.config));
        feeder =
            new Feeder(
                SerializerConstants.Feeder.config,
                new SimTalonFXIO(SerializerConstants.Feeder.config));
        climber = new Climber(ClimberConstants.config, new SimTalonFXIO(ClimberConstants.config));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        flywheels =
            new Flywheels(
                LauncherConstants.Flywheels.leftConfig,
                LauncherConstants.Flywheels.rightConfig,
                new MotorIO() {},
                new MotorIO() {});

        turret =
            new Turret(
                new TurretSubsystemConfig(), new TurretMotorIOSim(new TurretSubsystemConfig()));
        intakeRoller =
            new IntakeRoller(
                new TalonFXSubsystemConfig(), new SimTalonFXIO(new TalonFXSubsystemConfig()));
        intakeExtension =
            new IntakeExtension(
                new TalonFXSubsystemConfig(), new SimTalonFXIO(new TalonFXSubsystemConfig()));
        dyeRotor =
            new DyeRotor(
                new TalonFXSubsystemConfig(), new SimTalonFXIO(new TalonFXSubsystemConfig()));
        feeder =
            new Feeder(
                new TalonFXSubsystemConfig(), new SimTalonFXIO(new TalonFXSubsystemConfig()));
        climber =
            new Climber(
                new TalonFXSubsystemConfig(), new SimTalonFXIO(new TalonFXSubsystemConfig()));
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();

    // configure the kinematics calculations
    configureKinematics();
  }

  /** Use this robot to configure the transforms between subsystems. */
  private void configureKinematics() {
    kinematicsManager.registerUnpublished(drive, 0, -1);
    kinematicsManager.register(
        intakeExtension,
        IntakeConstants.Extension.MODEL_INDEX,
        IntakeConstants.Extension.PARENT_INDEX);
    kinematicsManager.register(
        dyeRotor,
        SerializerConstants.DyeRotor.MODEL_INDEX,
        SerializerConstants.DyeRotor.PARENT_INDEX);
    kinematicsManager.register(
        turret, LauncherConstants.Turret.MODEL_INDEX, LauncherConstants.Turret.PARENT_INDEX);
    kinematicsManager.register(
        hood, LauncherConstants.Hood.MODEL_INDEX, LauncherConstants.Hood.PARENT_INDEX);
    kinematicsManager.registerUnpublished(
        flywheels,
        LauncherConstants.Flywheels.MODEL_INDEX,
        LauncherConstants.Flywheels.PARENT_INDEX);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    driverControls.configureButtonBindings();
    driverControls.configureTriggers();
    // operatorControls.configureButtonBindings();
    // operatorControls.configureTriggers();
    // devControls.configureButtonBindings();
    // devControls.configureTriggers();

    // Default command, normal field-relative drive
    driverControls.setToNormalDrive();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
