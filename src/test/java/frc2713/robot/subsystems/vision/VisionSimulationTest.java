package frc2713.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc2713.robot.Constants;
import frc2713.robot.FieldConstants.AprilTagLayoutType;
import frc2713.robot.RobotContainer;
import frc2713.robot.generated.TunerConstants;
import frc2713.robot.subsystems.drive.Drive;
import frc2713.robot.subsystems.drive.GyroIO;
import frc2713.robot.subsystems.drive.ModuleIO;
import org.junit.jupiter.api.Test;

class VisionSimulationTest {
  @Test
  void simPublishesCameraObservationsAndKeepsTruthIndependentOfVision() {
    assertTrue(HAL.initialize(500, 0));
    boolean previousDisableHAL = Constants.disableHAL;
    Constants.disableHAL();
    var previousDrive = RobotContainer.drive;
    SimHooks.pauseTiming();
    Drive drive = null;
    Vision vision = null;
    try {
      var modules =
          new TestModule[] {
            new TestModule(TunerConstants.FrontLeft.WheelRadius),
            new TestModule(TunerConstants.FrontRight.WheelRadius),
            new TestModule(TunerConstants.BackLeft.WheelRadius),
            new TestModule(TunerConstants.BackRight.WheelRadius)
          };
      drive = new Drive(new GyroIO() {}, modules[0], modules[1], modules[2], modules[3]);
      RobotContainer.drive = drive;
      var tag = AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(1).orElseThrow();
      var truth =
          tag.transformBy(new Transform3d(2, 0, 0, new Rotation3d(0, 0, Math.PI))).toPose2d();
      drive.setPose(truth);
      var simDrive = drive;
      var io =
          VisionConstants.CAMERAS.stream()
              .map(
                  c ->
                      new VisionIOPhotonVisionSim(
                          c.name(),
                          c.robotToCamera(),
                          simDrive::getSimulationPose,
                          simDrive::getPose))
              .toArray(VisionIO[]::new);
      int[] observations = {0};
      vision =
          new Vision(
              (pose, timestamp, stdDevs) -> {
                observations[0]++;
                assertTrue(pose.getTranslation().getDistance(truth.getTranslation()) < 0.5);
                simDrive.addVisionMeasurement(pose, timestamp, stdDevs);
              },
              drive::getPose,
              drive::setPose,
              VisionConstants.CAMERAS,
              io);
      for (int i = 0; i < 100; i++) {
        SimHooks.stepTiming(0.02);
        drive.periodic();
        vision.periodic();
      }
      assertTrue(observations[0] > 0, "Sim must produce usable vision without a coprocessor");
      var nt = NetworkTableInstance.getDefault();
      for (var config : VisionConstants.CAMERAS) {
        String name = config.name();
        try (var heartbeat =
            nt.getTable("photonvision")
                .getSubTable(name)
                .getIntegerTopic("heartbeat")
                .subscribe(-1)) {
          assertTrue(heartbeat.get() > 0, "Sim must publish frames for " + name);
        }
      }

      // Correct the estimate deliberately; synthetic cameras must keep their independent truth.
      drive.addVisionMeasurement(
          truth.transformBy(new Transform2d(1, 0, Rotation2d.kZero)),
          Timer.getTimestamp(),
          VecBuilder.fill(0.0, 0.0, 0.0));
      assertTrue(drive.getPose().getTranslation().getDistance(truth.getTranslation()) > 0.9);
      assertEquals(truth, drive.getSimulationPose());

      // Wheel movement advances simulation truth, and explicit resets reposition both poses.
      for (var module : modules) module.distanceMeters = 1.0;
      SimHooks.stepTiming(0.02);
      drive.periodic();
      var movedTruth = truth.transformBy(new Transform2d(1, 0, Rotation2d.kZero));
      assertEquals(movedTruth.getX(), drive.getSimulationPose().getX(), 1e-9);
      assertEquals(movedTruth.getY(), drive.getSimulationPose().getY(), 1e-9);
      drive.setPose(Pose2d.kZero);
      assertEquals(Pose2d.kZero, drive.getPose());
      assertEquals(Pose2d.kZero, drive.getSimulationPose());
    } finally {
      if (vision != null) vision.close();
      if (drive != null) CommandScheduler.getInstance().unregisterSubsystem(drive);
      RobotContainer.drive = previousDrive;
      Constants.disableHAL = previousDisableHAL;
      SimHooks.resumeTiming();
    }
  }

  private static class TestModule implements ModuleIO {
    private final double wheelRadius;
    double distanceMeters;

    TestModule(double wheelRadius) {
      this.wheelRadius = wheelRadius;
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
      inputs.drivePositionRad = distanceMeters / wheelRadius;
      inputs.odometryTimestamps = new double[] {Timer.getTimestamp()};
      inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
      inputs.odometryTurnPositions = new Rotation2d[] {Rotation2d.kZero};
    }
  }
}
