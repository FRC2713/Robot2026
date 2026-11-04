package frc2713.robot.subsystems.vision;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

/** One recorder for all Jetson cameras. No NetworkTables writes in simulation or replay. */
final class RewindRecorder implements AutoCloseable {
  private final BooleanPublisher publisher;
  private final Consumer<Boolean> output;
  private boolean recording;
  private boolean teleopRan;
  private double lastEnabledTimestamp;

  RewindRecorder(boolean enabled) {
    publisher =
        enabled
            ? NetworkTableInstance.getDefault()
                .getTable("photonvision/rewind")
                .getBooleanTopic("record")
                .publish()
            : null;
    output = publisher == null ? null : publisher::set;
    if (output != null) output.accept(false);
  }

  RewindRecorder(Consumer<Boolean> output) {
    publisher = null;
    this.output = output;
    output.accept(false);
  }

  void periodic() {
    update(
        DriverStation.isEnabled(),
        DriverStation.isAutonomous(),
        DriverStation.isTeleop(),
        Timer.getTimestamp());
    Logger.recordOutput("Vision/RewindRecording", recording);
  }

  void update(boolean enabled, boolean autonomous, boolean teleop, double now) {
    if (output == null) return;
    if (!recording) {
      if (enabled && autonomous) {
        recording = true;
        teleopRan = false;
        lastEnabledTimestamp = now;
        output.accept(true);
      }
    } else if (enabled) {
      lastEnabledTimestamp = now;
      teleopRan |= teleop;
    } else if (teleopRan
        || now - lastEnabledTimestamp > VisionConstants.REWIND_DISABLED_TIMEOUT_SECONDS) {
      recording = false;
      output.accept(false);
    }
  }

  @Override
  public void close() {
    if (recording && output != null) output.accept(false);
    if (publisher != null) publisher.close();
  }
}
