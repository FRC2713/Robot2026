// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc2713.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import frc2713.lib.drivers.CANDeviceId;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final CANDeviceId launcherMotorDeviceId = new CANDeviceId(2);

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class Launcher {
    public static final Distance bigWheelRadius =
        Distance.ofBaseUnits(Units.inchesToMeters(2.0), Meters);
    public static final Distance smallWheelRadius =
        Distance.ofBaseUnits(Units.inchesToMeters(1.0), Meters);

    public static TalonFXSubsystemConfig launcherConfig = new TalonFXSubsystemConfig();

    static {
      launcherConfig.name = "Launcher";
      launcherConfig.talonCANID = launcherMotorDeviceId;
      launcherConfig.fxConfig.Slot0.kP = 0.1;
      launcherConfig.fxConfig.Slot0.kI = 0.0;
      launcherConfig.fxConfig.Slot0.kD = 0.0;
      launcherConfig.unitToRotorRatio = 1.0; // 1:1 ratio
    }
  }

  public static final class Turret {
    public static final Distance turretRadius =
        Distance.ofBaseUnits(Units.inchesToMeters(6.0), Meters);
  }

  public static final class Climber {
    public static final Distance armLength =
        Distance.ofBaseUnits(Units.inchesToMeters(12.0), Meters);
  }

  public static final class Indexer {
    public static final Distance wheelRadius =
        Distance.ofBaseUnits(Units.inchesToMeters(1.0), Meters);
  }

  public static final class Feeder {
    public static final Distance wheelRadius =
        Distance.ofBaseUnits(Units.inchesToMeters(1.0), Meters);
  }

  public static final class IntakeExtension {
    public static final Distance armLength =
        Distance.ofBaseUnits(Units.inchesToMeters(10.0), Meters);
  }

  public static final class IntakeRollers {
    public static final Distance wheelRadius =
        Distance.ofBaseUnits(Units.inchesToMeters(1.5), Meters);
  }
}
