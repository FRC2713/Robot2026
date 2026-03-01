// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc2713.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecondPerSecond;

import choreo.auto.AutoFactory;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc2713.lib.io.MotorIO;
import frc2713.lib.io.SimTalonFXIO;
import frc2713.lib.io.TalonFXIO;
import frc2713.lib.subsystem.KinematicsManager;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;
import frc2713.lib.util.AllianceFlipUtil;
import frc2713.robot.commands.DriveCommands;
import frc2713.robot.commands.autos.NeutralScoreNeutral;
import frc2713.robot.commands.autos.NeutralScoreOutpostOTF;
import frc2713.robot.commands.autos.RightSideAutoBump;
import frc2713.robot.generated.TunerConstants;
import frc2713.robot.oi.DevControls;
import frc2713.robot.oi.DriverControls;
import frc2713.robot.subsystems.drive.Drive;
import frc2713.robot.subsystems.drive.GyroIO;
import frc2713.robot.subsystems.drive.GyroIOPigeon2;
import frc2713.robot.subsystems.drive.ModuleIO;
import frc2713.robot.subsystems.drive.ModuleIOSim;
import frc2713.robot.subsystems.drive.ModuleIOTalonFX;
import frc2713.robot.subsystems.intake.IntakeConstants;
import frc2713.robot.subsystems.intake.IntakeExtension;
import frc2713.robot.subsystems.intake.IntakeExtensionIO;
import frc2713.robot.subsystems.intake.IntakeExtensionIOTalonFX;
import frc2713.robot.subsystems.intake.IntakeRoller;
import frc2713.robot.subsystems.launcher.Flywheels;
import frc2713.robot.subsystems.launcher.Hood;
import frc2713.robot.subsystems.launcher.LauncherConstants;
import frc2713.robot.subsystems.launcher.LaunchingSolutionManager;
import frc2713.robot.subsystems.launcher.Turret;
import frc2713.robot.subsystems.launcher.turretIO.TurretMotorIO;
import frc2713.robot.subsystems.launcher.turretIO.TurretMotorIOSim;
import frc2713.robot.subsystems.launcher.turretIO.TurretMotorIOTalonFX;
import frc2713.robot.subsystems.launcher.turretIO.TurretSubsystemConfig;
import frc2713.robot.subsystems.serializer.DyeRotor;
import frc2713.robot.subsystems.serializer.Feeder;
import frc2713.robot.subsystems.serializer.SerializerConstants;
import frc2713.robot.subsystems.vision.Vision;
import frc2713.robot.subsystems.vision.VisionIO;
import frc2713.robot.subsystems.vision.VisionIOSLAMDunk;
import java.util.Arrays;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  public static Drive drive;
  private static Flywheels flywheels;
  private static Turret turret;
  private static Hood hood;
  private static IntakeRoller intakeRoller;
  public static IntakeExtension intakeExtension;
  private static DyeRotor dyeRotor;
  private static Feeder feeder;
  public static Vision vision;

  // Lazy loaders
  @SuppressWarnings("unused")
  private final KinematicsManager kinematicsManager = new KinematicsManager();

  @SuppressWarnings("unused")
  private final LaunchingSolutionManager launchingSolutionManager = new LaunchingSolutionManager();

  // Controllers
  public static DriverControls driverControls;
  public static DevControls devControls;

  // Dashboard inputs
  private final AutoFactory autoFactory;
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
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        // drive =
        //     new Drive(
        //         new GyroIO() {},
        //         new ModuleIO() {},
        //         new ModuleIO() {},
        //         new ModuleIO() {},
        //         new ModuleIO() {});
        flywheels =
            new Flywheels(
                LauncherConstants.Flywheels.leaderConfig,
                LauncherConstants.Flywheels.followerConfig,
                new TalonFXIO(LauncherConstants.Flywheels.leaderConfig),
                new TalonFXIO(LauncherConstants.Flywheels.followerConfig));
        hood =
            new Hood(LauncherConstants.Hood.config, new TalonFXIO(LauncherConstants.Hood.config));
        turret =
            new Turret(
                LauncherConstants.Turret.config,
                new TurretMotorIOTalonFX(LauncherConstants.Turret.config));

        intakeRoller =
            new IntakeRoller(
                IntakeConstants.Roller.leaderConfig,
                IntakeConstants.Roller.followerConfig,
                new TalonFXIO(IntakeConstants.Roller.leaderConfig),
                new TalonFXIO(IntakeConstants.Roller.followerConfig));

        intakeExtension =
            new IntakeExtension(
                IntakeConstants.Extension.config,
                new IntakeExtensionIOTalonFX(IntakeConstants.Extension.differentialConfig));

        dyeRotor =
            new DyeRotor(
                SerializerConstants.DyeRotor.config,
                new TalonFXIO(SerializerConstants.DyeRotor.config));
        feeder =
            new Feeder(
                SerializerConstants.Feeder.config,
                new TalonFXIO(SerializerConstants.Feeder.config));
        vision = new Vision(new VisionIO() {});
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
                LauncherConstants.Flywheels.leaderConfig,
                LauncherConstants.Flywheels.followerConfig,
                new SimTalonFXIO(LauncherConstants.Flywheels.leaderConfig),
                new SimTalonFXIO(LauncherConstants.Flywheels.followerConfig));
        hood =
            new Hood(
                LauncherConstants.Hood.config, new SimTalonFXIO(LauncherConstants.Hood.config));
        turret =
            new Turret(
                LauncherConstants.Turret.config,
                new TurretMotorIOSim(LauncherConstants.Turret.config));
        intakeRoller =
            new IntakeRoller(
                IntakeConstants.Roller.leaderConfig,
                IntakeConstants.Roller.followerConfig,
                new SimTalonFXIO(IntakeConstants.Roller.leaderConfig),
                new SimTalonFXIO(IntakeConstants.Roller.followerConfig));
        // intakeExtension =
        //     new IntakeExtension(
        //         IntakeConstants.Extension.config,
        //         new SimTalonFXIO(IntakeConstants.Extension.config));
        dyeRotor =
            new DyeRotor(
                SerializerConstants.DyeRotor.config,
                new SimTalonFXIO(SerializerConstants.DyeRotor.config));
        feeder =
            new Feeder(
                SerializerConstants.Feeder.config,
                new SimTalonFXIO(SerializerConstants.Feeder.config));
        vision = new Vision(new VisionIOSLAMDunk() {});
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
                LauncherConstants.Flywheels.leaderConfig,
                LauncherConstants.Flywheels.followerConfig,
                new MotorIO() {},
                new MotorIO() {});
        hood = new Hood(new TalonFXSubsystemConfig(), new MotorIO() {});
        turret = new Turret(new TurretSubsystemConfig(), new TurretMotorIO() {});
        intakeRoller =
            new IntakeRoller(
                new TalonFXSubsystemConfig(),
                new TalonFXSubsystemConfig(),
                new MotorIO() {},
                new MotorIO() {});
        intakeExtension =
            new IntakeExtension(new TalonFXSubsystemConfig(), new IntakeExtensionIO() {});
        dyeRotor = new DyeRotor(new TalonFXSubsystemConfig(), new MotorIO() {});
        feeder = new Feeder(new TalonFXSubsystemConfig(), new MotorIO() {});
        vision = new Vision(new VisionIO() {});
        break;
    }
    //
    autoFactory =
        new AutoFactory(
            drive::getPose, // Function that returns the current robot pose
            drive::setPose, // Function that resets the current robot pose to the provided Pose2d
            drive::followTrajectory, // The drive subsystem trajectory follower
            true, // If alliance flipping should be enabled
            drive,
            (sample, isStart) -> {
              Logger.recordOutput(
                  "TrajectoryFollowing/ActiveTrajectory",
                  Arrays.stream(sample.getPoses())
                      .map(AllianceFlipUtil::apply)
                      .toArray(Pose2d[]::new));
            } // The drive subsystem
            );

    // This setting is ignored when the FMS is connected
    DriverStation.silenceJoystickConnectionWarning(true);

    // Set up controllers
    driverControls =
        new DriverControls(
            drive, flywheels, turret, hood, intakeRoller, intakeExtension, dyeRotor, feeder);
    devControls =
        new DevControls(
            drive, flywheels, turret, hood, intakeRoller, intakeExtension, dyeRotor, feeder);

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
    autoChooser.addOption(
        "Right Side Bump Auto",
        RightSideAutoBump.routine(
            autoFactory,
            drive,
            intakeExtension,
            intakeRoller,
            flywheels,
            hood,
            turret,
            dyeRotor,
            feeder));
    autoChooser.addOption(
        "Trench to neutral launch then refuel",
        NeutralScoreNeutral.routine(
            autoFactory,
            drive,
            intakeExtension,
            intakeRoller,
            flywheels,
            hood,
            turret,
            dyeRotor,
            feeder,
            () -> GameCommandGroups.getOtfShot(flywheels, hood, turret, feeder, dyeRotor)));
    autoChooser.addOption(
        "Trench to neutral otf to outpost",
        NeutralScoreOutpostOTF.routine(
            autoFactory,
            drive,
            intakeExtension,
            intakeRoller,
            flywheels,
            hood,
            turret,
            dyeRotor,
            feeder,
            () -> GameCommandGroups.getOtfShot(flywheels, hood, turret, feeder, dyeRotor)));

    // Configure the button bindings
    configureButtonBindings();

    // configure the kinematics calculations
    configureKinematics();
  }

  /** Use this robot to configure the transforms between subsystems. */
  private void configureKinematics() {
    KinematicsManager.getInstance().registerUnpublished(drive, 0, -1);
    KinematicsManager.getInstance()
        .register(
            intakeExtension,
            IntakeConstants.Extension.MODEL_INDEX,
            IntakeConstants.Extension.PARENT_INDEX);
    KinematicsManager.getInstance()
        .register(
            dyeRotor,
            SerializerConstants.DyeRotor.MODEL_INDEX,
            SerializerConstants.DyeRotor.PARENT_INDEX);
    KinematicsManager.getInstance()
        .register(
            turret, LauncherConstants.Turret.MODEL_INDEX, LauncherConstants.Turret.PARENT_INDEX);
    KinematicsManager.getInstance()
        .register(hood, LauncherConstants.Hood.MODEL_INDEX, LauncherConstants.Hood.PARENT_INDEX);
    KinematicsManager.getInstance()
        .registerUnpublished(
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
    driverControls.configureButtonBindings();
    devControls.configureButtonBindings();

    // Default commands
    // Set drive command to accept inputs from both driver and dev controllers
    DriveCommands.setDefaultDriveCommand(
        drive,
        DriveCommands.joystickDrive(
            drive,
            () -> -driverControls.getLeftY() + -devControls.getLeftY(),
            () -> -driverControls.getLeftX() + -devControls.getLeftX(),
            () -> -driverControls.getRightX() + -devControls.getRightX()),
        "Dual Controller Drive");
    // turret.setDefaultCommand(turret.otfCommand().withName("OTF Tracking"));
    // hood.setDefaultCommand(
    //     hood.autoRetractCommand(drive::getPose, hood.otfAngSupplier)
    //         .withName("OTF Tracking with Auto-Retract"));
    // flywheels.setDefaultCommand(flywheels.idleSpeedCommand().withName("Idle Tracking"));

    // Comment these out when using dev controller
    // driverControls.setToNormalDrive();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /**
   * A Utility class holding common game actions in the form of command groups that can be shared
   * between Driver, Developer, and AutoRoutines
   */
  public static class GameCommandGroups {
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
        DyeRotor dyeRotor) {
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
              dyeRotor.feedWhenReady(flywheels::atTarget))
          .withName("OTF Shooting");
    }

    public static Command dumbShot(
        Drive drive,
        Flywheels flywheels,
        Hood hood,
        Turret turret,
        Feeder feeder,
        DyeRotor dyeRotor) {
      return Commands.parallel(
          flywheels.setVelocity(() -> LauncherConstants.Flywheels.staticTowerVelocity),
          // hood.setAngleCommand(() -> Degrees.of(0.0)),
          // turret.setAngle(() -> Degrees.of(0.0)),
          feeder.feedWhenReady(() -> flywheels.atTarget()),
          dyeRotor.feedWhenReady(() -> flywheels.atTarget()));
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
              flywheels.simulateLaunchedFuel(() -> flywheels.atTarget() && hood.atTarget()),
              feeder.feedWhenReady(() -> flywheels.atTarget() && hood.atTarget()),
              dyeRotor.feedWhenReady(() -> flywheels.atTarget() && hood.atTarget()))
          .withName("Hub Shooting");
    }

    /** Stop shooting and clear drive limits. */
    public static Command stopShooting(Drive drive, Feeder feeder, DyeRotor dyeRotor) {
      return Commands.parallel(
              DriveCommands.clearDriveLimits(drive), feeder.stop(), dyeRotor.stopCommand())
          .withName("Stopped Shooting");
    }
  }
}
