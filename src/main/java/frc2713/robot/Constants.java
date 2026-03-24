// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc2713.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc2713.lib.drivers.CANDeviceId;
// import frc2713.robot.generated.BuildConstants;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  // sets default behaviour to track OTF solutions
  public static final boolean enableOTFFeatures = true; // currentMode == Mode.SIM;

  public static final CANDeviceId launcherMotorDeviceId = new CANDeviceId(2);

  // Set this to true to enable tunable numbers for all subsystems
  // Based off event branch: !BuildConstants.GIT_BRANCH.startsWith("event");
  public static boolean tuningMode = true;

  // Set this to true to log performance data for each subsystem
  public static boolean logPerformanceData = true;

  // Set this to true to log custom power consumption data
  public static boolean logPowerData = true;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }
}
