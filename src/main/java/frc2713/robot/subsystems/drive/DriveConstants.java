package frc2713.robot.subsystems.drive;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import frc2713.lib.util.LoggedTunableGains;
import frc2713.lib.util.LoggedTunableNumber;

public class DriveConstants {
  public final class AutoConstants {
    public static final LoggedTunableGains xTrajectoryController =
        new LoggedTunableGains("xTraj", new Slot0Configs().withKP(2), new MotionMagicConfigs());
    public static final LoggedTunableGains yTrajectoryController =
        new LoggedTunableGains("yTraj", new Slot0Configs().withKP(2), new MotionMagicConfigs());
    public static final LoggedTunableGains headingTrajectoryController =
        new LoggedTunableGains(
            "headingTraj", new Slot0Configs().withKP(3).withKD(0.0), new MotionMagicConfigs());
  }

  public static final LoggedTunableNumber speedScalar =
      new LoggedTunableNumber("Drive/Drive Speed Scalar", 1);
}
