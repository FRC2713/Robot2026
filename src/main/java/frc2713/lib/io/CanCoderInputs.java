package frc2713.lib.io;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class CanCoderInputs {
  /** Absolute position in rotations [0, 1) */
  public Angle absolutePosition = null;

  /** Velocity in rotations per second */
  public AngularVelocity velocity = RotationsPerSecond.of(0.0);
}
