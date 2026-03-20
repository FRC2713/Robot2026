package frc2713.robot.util;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc2713.robot.FieldConstants;
import frc2713.robot.GamePieceConstants;
import frc2713.robot.subsystems.launcher.LauncherConstants;

/**
 * Seeds distance → time-of-flight tables by integrating {@link BallTrajectorySim} at each LUT
 * distance using RPM/hood lookup → {@link LauncherConstants.Flywheels} / {@link
 * LauncherConstants.Hood}.
 */
public final class LaunchTofTable {

  private static final double SIM_DT = 0.001;
  private static final double HIT_TOLERANCE_M = 0.40;
  private static final int MAX_STEPS = 20000;

  private LaunchTofTable() {}

  private static final double[] SCORING_DISTANCES_M = {1.03, 2.1, 3.36, 5.0, 6.03};

  private static final double[] AZ_EXTRA_DISTANCES_M = {2.11, 6.44};

  public static void seedScoringTofMap(
      InterpolatingDoubleTreeMap dest,
      InterpolatingDoubleTreeMap rpmMap,
      InterpolatingDoubleTreeMap hoodMap,
      double muzzleHeightM) {
    for (double d : SCORING_DISTANCES_M) {
      dest.put(d, simulateTimeOfFlightMeters(d, rpmMap, hoodMap, muzzleHeightM));
    }
  }

  public static void seedAllianceZoneTofMap(
      InterpolatingDoubleTreeMap dest,
      InterpolatingDoubleTreeMap rpmMap,
      InterpolatingDoubleTreeMap hoodMap,
      double muzzleHeightM) {
    for (double d : SCORING_DISTANCES_M) {
      dest.put(d, simulateTimeOfFlightMeters(d, rpmMap, hoodMap, muzzleHeightM));
    }
    for (double d : AZ_EXTRA_DISTANCES_M) {
      dest.put(d, simulateTimeOfFlightMeters(d, rpmMap, hoodMap, muzzleHeightM));
    }
  }

  static double simulateTimeOfFlightMeters(
      double horizontalDistanceM,
      InterpolatingDoubleTreeMap rpmMap,
      InterpolatingDoubleTreeMap hoodMap,
      double muzzleHeightM) {
    double hubZ = FieldConstants.Hub.topCenterPoint.getZ();
    double rpm = rpmMap.get(horizontalDistanceM);
    double hoodDeg = hoodMap.get(horizontalDistanceM);
    double v = LauncherConstants.Flywheels.muzzleVelocityMetersPerSecond(rpm);
    double theta = LauncherConstants.Hood.exitAngleRadiansFromHoodDegrees(hoodDeg);
    double cosT = Math.cos(theta);
    if (cosT < 0.05 || v < 1e-3) {
      return vacuumHorizontalTimeOfFlight(horizontalDistanceM, v, theta);
    }

    Translation3d start = new Translation3d(0, 0, muzzleHeightM);
    Translation3d vel = new Translation3d(v * cosT, 0, v * Math.sin(theta));
    Translation3d target = new Translation3d(horizontalDistanceM, 0, hubZ);

    BallTrajectorySim.Ball ball =
        new BallTrajectorySim.Ball(
            GamePieceConstants.Fuel.mass,
            GamePieceConstants.Fuel.radius,
            GamePieceConstants.Fuel.dragCoeff,
            GamePieceConstants.Fuel.liftCoeff);
    ball.launch(start, vel, RadiansPerSecond.of(0));

    double t = 0;
    for (int i = 0; i < MAX_STEPS; i++) {
      ball.update(Seconds.of(SIM_DT));
      t += SIM_DT;
      if (ball.getPosition().getDistance(target) < HIT_TOLERANCE_M) {
        return t;
      }
      if (t > 10.0 || ball.getPosition().getZ() < 0) {
        break;
      }
    }
    return vacuumHorizontalTimeOfFlight(horizontalDistanceM, v, theta);
  }

  private static double vacuumHorizontalTimeOfFlight(double d, double v, double theta) {
    double denom = v * Math.cos(theta);
    return denom > 1e-6 ? d / denom : 0.75;
  }
}
