package frc2713.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc2713.robot.Constants;
import frc2713.robot.subsystems.vision.VisionConstants.CameraConfig;
import frc2713.robot.subsystems.vision.VisionIO.PoseObservation;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

class VisionReplayTest {
  @Test
  void replayRestoresInputsAndDisableControlBeforeFusionAndRecreatesVisualization() {
    assertTrue(HAL.initialize(500, 0));
    Constants.disableHAL();
    var input = new VisionIOInputsAutoLogged();
    input.connected = true;
    input.tagIds = new int[] {1};
    input.poseObservations =
        new PoseObservation[] {
          new PoseObservation(1.0, new Pose3d(2, 2, 0, Rotation3d.kZero), 0, 2, 2)
        };
    double[] now = {1.1};
    boolean[] disabled = {false};
    var entry = new AtomicReference<LogTable>();
    List<Pose2d> received = new ArrayList<>();
    var mount = new Transform3d(0.3, -0.1, 0.5, new Rotation3d(0, -0.2, 0));
    var robot = new Pose2d(3, 2, Rotation2d.fromDegrees(90));
    Logger.AdvancedHooks.disableRobotBaseCheck();
    Logger.disableConsoleCapture();
    Logger.setReplaySource(
        table -> {
          table.setTimestamp((long) (now[0] * 1e6));
          input.toLog(table.getSubtable("Vision/replay-camera"));
          table.getSubtable("NetworkInputs").put("Vision/disable", disabled[0]);
          entry.set(table);
          return true;
        });
    try (var vision =
        new Vision(
            (pose, time, std) -> received.add(pose),
            () -> robot,
            pose -> {},
            List.of(new CameraConfig("replay-camera", mount, 1)),
            new VisionIO() {})) {
      // Unlike a direct LogTable round-trip, this uses Logger.processInputs and replay time.
      Logger.start();
      vision.periodic();
      assertTrue(vision.isConnected());
      assertEquals(List.of(input.poseObservations[0].pose().toPose2d()), received);
      var outputs = entry.get().getSubtable("ReplayOutputs");
      assertEquals(
          new Pose3d(robot).transformBy(mount),
          outputs.get("Vision/replay-camera/CameraPose", Pose3d.kZero));
      assertEquals(
          new Pose3d().transformBy(mount),
          outputs.get("Vision/replay-camera/RobotToCameraPose", Pose3d.kZero));
      assertEquals(1, outputs.get("Vision/Summary/TagPoses", new Pose3d[0]).length);
      assertTrue(outputs.get("Vision/Applying", false));

      now[0] = 1.12;
      disabled[0] = true;
      input.poseObservations =
          new PoseObservation[] {
            new PoseObservation(1.02, new Pose3d(2.1, 2, 0, Rotation3d.kZero), 0, 2, 2)
          };
      Logger.AdvancedHooks.invokePeriodicBeforeUser();
      vision.periodic();
      assertEquals(1, received.size());
      assertEquals(1, outputs.get("Vision/Summary/RobotPosesAccepted", new Pose3d[0]).length);
      assertFalse(outputs.get("Vision/Applying", true));
      assertTrue(outputs.get("Vision/Connected", false));

      disabled[0] = false;
      Logger.AdvancedHooks.invokePeriodicBeforeUser();
      vision.periodic();
      assertEquals(1, received.size(), "Re-enabling must not reapply the disabled frame");
      input.poseObservations =
          new PoseObservation[] {
            new PoseObservation(1.04, new Pose3d(2.2, 2, 0, Rotation3d.kZero), 0, 2, 2)
          };
      Logger.AdvancedHooks.invokePeriodicBeforeUser();
      vision.periodic();
      assertEquals(2, received.size());

      input.poseObservations = new PoseObservation[0];
      input.tagIds = new int[0];
      Logger.AdvancedHooks.invokePeriodicBeforeUser();
      vision.periodic();
      assertEquals(0, outputs.get("Vision/Summary/TagPoses", new Pose3d[0]).length);
      assertEquals(0, outputs.get("Vision/Summary/RobotPosesAccepted", new Pose3d[0]).length);
    } finally {
      Logger.end();
      Logger.setReplaySource(null);
    }
  }
}
