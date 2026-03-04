package frc2713.lib.io;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class CanCoderInputs {

  public Angle position;

  /** Absolute position in rotations [0, 1) */
  public Angle absolutePosition;

  /** Velocity in rotations per second */
  public AngularVelocity velocity;
}
