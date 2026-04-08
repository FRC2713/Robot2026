package frc2713.robot.subsystems.drive;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecondPerSecond;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc2713.lib.dynamics.MoiUnits;
import frc2713.lib.util.DriveLimits;
import frc2713.lib.util.LoggedTunableGains;
import frc2713.lib.util.LoggedTunableNumber;

public class DriveConstants {
  public final class AutoConstants {
    public static final LoggedTunableGains positionTrajectoryController =
        new LoggedTunableGains(
            "positionTraj", new Slot0Configs().withKP(1.8).withKD(0.0), new MotionMagicConfigs());
    public static final LoggedTunableGains headingTrajectoryController =
        new LoggedTunableGains(
            "headingTraj", new Slot0Configs().withKP(6.5).withKD(0.2), new MotionMagicConfigs());
    public static final LoggedTunableGains crosstrackTrajectoryController =
        new LoggedTunableGains(
            "crosstrackTraj", new Slot0Configs().withKP(1.0).withKD(0.0), new MotionMagicConfigs());
  }

  public static final LoggedTunableNumber speedScalar =
      new LoggedTunableNumber("Drive/Drive Speed Scalar", 1);

  public static final DriveLimits scoringDriveLimits =
      new DriveLimits(
          FeetPerSecond.of(3.0),
          FeetPerSecondPerSecond.of(11.0),
          DegreesPerSecond.of(45.0),
          DegreesPerSecondPerSecond.of(180.0));

  public static final DriveLimits feedingDriveLimits =
      new DriveLimits(
          FeetPerSecond.of(5.0),
          FeetPerSecondPerSecond.of(15.0),
          DegreesPerSecond.of(90.0),
          DegreesPerSecondPerSecond.of(360.0));

  public static final MomentOfInertia intakeExtendedMoi =
      MoiUnits.PoundSquareInches.of(29546.954784);
  public static final MomentOfInertia intakeRetractedMoi =
      MoiUnits.PoundSquareInches.of(25819.326619);
}
