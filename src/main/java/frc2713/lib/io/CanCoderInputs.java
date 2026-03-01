package frc2713.lib.io;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class CanCoderInputs {
  /** Absolute position in rotations [0, 1) */
  public double absolutePositionRotations = Double.NaN;

  /** Velocity in rotations per second */
  public double velocityRotationsPerSecond = 0.0;
}
