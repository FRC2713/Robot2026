package frc2713.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class RewindRecorderTest {
  @Test
  void recordsAutoThroughGapAndStopsAfterTeleopWithoutRepeatedWrites() {
    var writes = new ArrayList<Boolean>();
    try (var recorder = new RewindRecorder(writes::add)) {
      recorder.update(false, true, false, 0);
      recorder.update(true, true, false, 1);
      recorder.update(true, true, false, 15);
      recorder.update(false, false, true, 16);
      recorder.update(true, false, true, 18);
      recorder.update(true, false, true, 150);
      recorder.update(false, false, true, 151);
      recorder.update(false, false, true, 152);
      assertEquals(List.of(false, true, false), writes);
    }
  }

  @Test
  void autoOnlyTimesOutAndLaterAutoCanStartAgain() {
    var writes = new ArrayList<Boolean>();
    try (var recorder = new RewindRecorder(writes::add)) {
      recorder.update(true, true, false, 1);
      recorder.update(false, true, false, 5);
      assertEquals(List.of(false, true), writes);
      recorder.update(false, true, false, 12);
      recorder.update(true, false, true, 13);
      assertEquals(List.of(false, true, false), writes);
      recorder.update(true, true, false, 20);
      assertEquals(List.of(false, true, false, true), writes);
    }
    assertEquals(List.of(false, true, false, true, false), writes);
  }

  @Test
  void teleopOnlyDoesNotStartRecording() {
    var writes = new ArrayList<Boolean>();
    try (var recorder = new RewindRecorder(writes::add)) {
      recorder.update(true, false, true, 1);
      recorder.update(false, false, true, 2);
    }
    assertEquals(List.of(false), writes);
  }
}
