package frc2713.robot;

import choreo.auto.AutoFactory;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc2713.lib.io.CanCoderIO;
import frc2713.lib.io.CanCoderIOHardware;
import frc2713.lib.io.CanCoderInputs;
import frc2713.lib.io.CanCoderInputsAutoLogged;
import frc2713.lib.io.MotorIO;
import frc2713.lib.io.SimTalonFXIO;
import frc2713.lib.io.TalonFXIO;
import frc2713.lib.subsystem.KinematicsManager;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;
import frc2713.lib.util.AllianceFlipUtil;
import frc2713.robot.commands.DriveCommands;
import frc2713.robot.commands.autos.BLineDepotOnly;
import frc2713.robot.commands.autos.BLineMidwarsBackwards;
import frc2713.robot.commands.autos.BLineMidwarsConservative;
import frc2713.robot.commands.autos.BLineMidwarsOvercenter;
import frc2713.robot.commands.autos.BLineMidwarsTrenchified;
import frc2713.robot.commands.autos.BLineSweepAndOutpost;
import frc2713.robot.commands.autos.BLineTuning;
import frc2713.robot.commands.autos.BumpTest;
import frc2713.robot.commands.autos.Demo;
import frc2713.robot.commands.autos.DriveTest;
import frc2713.robot.generated.TunerConstants;
import frc2713.robot.oi.DevControls;
import frc2713.robot.oi.DriverControls;
import frc2713.robot.oi.OperatorControls;
import frc2713.robot.subsystems.drive.Drive;
import frc2713.robot.subsystems.drive.DriveConstants;
import frc2713.robot.subsystems.drive.GyroIO;
import frc2713.robot.subsystems.drive.GyroIOPigeon2;
import frc2713.robot.subsystems.drive.ModuleIO;
import frc2713.robot.subsystems.drive.ModuleIOSim;
import frc2713.robot.subsystems.drive.ModuleIOTalonFX;
import frc2713.robot.subsystems.intake.IntakeConstants;
import frc2713.robot.subsystems.intake.IntakeExtension;
import frc2713.robot.subsystems.intake.IntakeRoller;
import frc2713.robot.subsystems.intake.intakeExtensionIO.IntakeExtensionIO;
import frc2713.robot.subsystems.intake.intakeExtensionIO.IntakeExtensionIOSim;
import frc2713.robot.subsystems.intake.intakeExtensionIO.IntakeExtensionIOTalonFX;
import frc2713.robot.subsystems.launcher.Flywheels;
import frc2713.robot.subsystems.launcher.Hood;
import frc2713.robot.subsystems.launcher.LauncherConstants;
import frc2713.robot.subsystems.launcher.LaunchingSolutionManager;
import frc2713.robot.subsystems.launcher.Turret;
import frc2713.robot.subsystems.launcher.TurretSim;
import frc2713.robot.subsystems.serializer.DyeRotor;
import frc2713.robot.subsystems.serializer.Feeder;
import frc2713.robot.subsystems.serializer.SerializerConstants;
import frc2713.robot.subsystems.vision.Vision;
import frc2713.robot.subsystems.vision.VisionIO;
import frc2713.robot.subsystems.vision.VisionIOSLAMDunk;
import java.util.Arrays;
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
  public static Flywheels flywheels;
  public static Turret turret;
  public static Hood hood;
  public static IntakeRoller intakeRoller;
  public static IntakeExtension intakeExtension;
  public static DyeRotor dyeRotor;
  public static Feeder feeder;
  public static Vision vision;

  // Lazy loaders
  @SuppressWarnings("unused")
  private final KinematicsManager kinematicsManager = new KinematicsManager();

  @SuppressWarnings("unused")
  private final LaunchingSolutionManager launchingSolutionManager = new LaunchingSolutionManager();

  // Controllers
  public static DriverControls driverControls;
  public static OperatorControls operatorControls;
  public static DevControls devControls;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private static AutoFactory choreoFactory;
  public static FollowPath.Builder pathBuilder;

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
                new TalonFXIO(LauncherConstants.Turret.config),
                new CanCoderInputsAutoLogged(),
                new CanCoderIOHardware(LauncherConstants.Turret.canCoderConfig));

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
        // vision = new Vision(new VisionIOSLAMDunk());
        vision = new Vision(new VisionIO() {}); // REVERT THIS

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

        turret = new TurretSim();

        intakeRoller =
            new IntakeRoller(
                IntakeConstants.Roller.leaderConfig,
                IntakeConstants.Roller.followerConfig,
                new SimTalonFXIO(IntakeConstants.Roller.leaderConfig),
                new SimTalonFXIO(IntakeConstants.Roller.followerConfig));
        intakeExtension =
            new IntakeExtension(
                IntakeConstants.Extension.config,
                new IntakeExtensionIOSim(IntakeConstants.Extension.config));
        dyeRotor =
            new DyeRotor(
                SerializerConstants.DyeRotor.config,
                new SimTalonFXIO(SerializerConstants.DyeRotor.config));
        feeder =
            new Feeder(
                SerializerConstants.Feeder.config,
                new SimTalonFXIO(SerializerConstants.Feeder.config));

        vision = new Vision(new VisionIOSLAMDunk()); // if jetson is connected to roboRio
        // vision = new Vision(new VisionIOLocalNT()); // if jetson is connected to laptop
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
        turret =
            new Turret(
                LauncherConstants.Turret.config,
                new MotorIO() {},
                new CanCoderInputsAutoLogged(),
                new CanCoderIO() {
                  @Override
                  public void readInputs(CanCoderInputs inputs) {}
                });
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

    // This setting is ignored when the FMS is connected
    DriverStation.silenceJoystickConnectionWarning(true);

    // Set up controllers
    driverControls =
        new DriverControls(
            drive,
            flywheels,
            turret,
            hood,
            intakeRoller,
            intakeExtension,
            dyeRotor,
            feeder,
            vision);
    devControls =
        new DevControls(
            drive, flywheels, turret, hood, intakeRoller, intakeExtension, dyeRotor, feeder);
    operatorControls =
        new OperatorControls(
            drive, flywheels, turret, hood, intakeRoller, intakeExtension, dyeRotor, feeder);

    // Set up auto routines
    configurePIDPathBuilder(Constants.tuningMode);
    configureChoreoFactory();

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    configureAutonomousRoutines(autoChooser, Constants.tuningMode);

    // Configure the button bindings
    configureButtonBindings();

    // configure the kinematics calculations
    configureKinematics();

    Path.setDefaultGlobalConstraints(
        new Path.DefaultGlobalConstraints(4.5, 12.0, 540, 860, 0.03, 2.0, 0.2));

    SmartDashboard.putNumber("autoStartDelay", 0.0);
  }

  private void configureChoreoFactory() {
    RobotContainer.choreoFactory =
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
  }

  private void configurePIDPathBuilder(boolean isDev) {
    // Path following
    RobotContainer.pathBuilder =
        new FollowPath.Builder(
                drive,
                drive::getPose,
                drive::getChassisSpeeds,
                drive::runVelocity,
                new PIDController(
                    DriveConstants.AutoConstants.positionTrajectoryController.getP().get(),
                    DriveConstants.AutoConstants.positionTrajectoryController.getI().get(),
                    DriveConstants.AutoConstants.positionTrajectoryController.getD().get()),
                new PIDController(
                    DriveConstants.AutoConstants.headingTrajectoryController.getP().get(),
                    DriveConstants.AutoConstants.headingTrajectoryController.getI().get(),
                    DriveConstants.AutoConstants.headingTrajectoryController.getD().get()),
                new PIDController(
                    DriveConstants.AutoConstants.crosstrackTrajectoryController.getP().get(),
                    DriveConstants.AutoConstants.crosstrackTrajectoryController.getI().get(),
                    DriveConstants.AutoConstants.crosstrackTrajectoryController.getD().get()))
            .withDefaultShouldFlip();

    // Bline logging
    FollowPath.setTranslationListLoggingConsumer(
        pair -> {
          Logger.recordOutput(pair.getFirst(), pair.getSecond());
        });

    if (isDev) {
      FollowPath.setDoubleLoggingConsumer(
          pair -> {
            Logger.recordOutput(pair.getFirst(), pair.getSecond());
          });

      FollowPath.setBooleanLoggingConsumer(
          pair -> {
            Logger.recordOutput(pair.getFirst(), pair.getSecond());
          });

      FollowPath.setPoseLoggingConsumer(
          pair -> {
            Logger.recordOutput(pair.getFirst(), pair.getSecond());
          });
    }
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
    operatorControls.configureButtonBindings();

    // Default commands
    // Set drive command to accept inputs from both driver and dev controllers
    // DriveCommands.setDefaultDriveCommand(
    //     drive,
    //     DriveCommands.joystickDrive(
    //         drive,
    //         () -> -driverControls.getLeftY() + -devControls.getLeftY(),
    //         () -> -driverControls.getLeftX() + -devControls.getLeftX(),
    //         () -> -driverControls.getRightX() + -devControls.getRightX()),
    //     "Dual Controller Drive");
    driverControls.setToNormalDrive();

    // Comment these out when using dev controller
    // driverControls.setToNormalDrive();
  }

  /**
   * Configure the autonomous chooser. Pass isDeve=true to add development auto routines.
   *
   * @param autoChooser
   * @param isDev pass true if not at a competition. TODO: Make this configurable
   */
  private void configureAutonomousRoutines(
      LoggedDashboardChooser<Command> autoChooser, boolean isDev) {

    if (isDev) {
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

      autoChooser.addOption("DriveTest", DriveTest.routine(choreoFactory));
      autoChooser.addOption("DemoMode", Demo.demo());
      autoChooser.addOption("BLine Tuning", BLineTuning.getCommand());
      autoChooser.addOption("Bump Test", BumpTest.getCommand());
    }

    // Competition Autos
    autoChooser.addDefaultOption("Midwars - R", BLineMidwarsOvercenter.getCommand(() -> false));
    autoChooser.addOption("Midwars - L", BLineMidwarsOvercenter.getCommand(() -> true));

    autoChooser.addOption(
        "Midwars - Inside Out - R", BLineMidwarsBackwards.getCommand(() -> false));
    autoChooser.addOption("Midwars - Inside Out - L", BLineMidwarsBackwards.getCommand(() -> true));

    autoChooser.addOption(
        "Midwars - Trenchified - R", BLineMidwarsTrenchified.getCommand(() -> false));
    autoChooser.addOption(
        "Midwars - Trenchified - L", BLineMidwarsTrenchified.getCommand(() -> true));

    autoChooser.addOption(
        "Midwars - Conservative - R", BLineMidwarsConservative.getCommand(() -> false));
    autoChooser.addOption(
        "Midwars - Conservative - L", BLineMidwarsConservative.getCommand(() -> true));
    autoChooser.addOption("Sweep And Outpost", BLineSweepAndOutpost.getCommand());
    autoChooser.addOption("Hub to Depot to Midline", BLineDepotOnly.getCommand());

    // autoChooser.addOption("NoIntake - R", NoIntake.getRoutine(choreoFactory, false, drive));
    // autoChooser.addOption("NoIntake - L", NoIntake.getRoutine(choreoFactory, true, drive));
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
