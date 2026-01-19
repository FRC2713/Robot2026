package frc2713.lib.util;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;

public class RobotTime {
  public static Time getTimestamp() {
    return Seconds.of(Timer.getFPGATimestamp());
  }
}
