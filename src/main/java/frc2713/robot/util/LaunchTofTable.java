package frc2713.robot.util;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc2713.robot.FieldConstants;
import frc2713.robot.GamePieceConstants;
import frc2713.robot.subsystems.launcher.LauncherConstants;
import java.util.ArrayList;
import java.util.Locale;
import org.littletonrobotics.junction.Logger;

/**
 * Seeds distance → time-of-flight tables by integrating {@link BallTrajectorySim} at each LUT
 * distance using RPM/hood lookup → {@link LauncherConstants.Flywheels} / {@link
 * LauncherConstants.Hood}. Distances whose simulation fails are omitted from the map and reported
 * via {@link DriverStation} and {@link Alert}.
 *
 * <p>Each seed attempt logs sampled ball positions to AdvantageKit under {@code
 * LaunchTofTable/seed/d_<m>p<frac>/} ({@code ball_t_s}, {@code ball_x_m}, …) plus {@code target},
 * {@code outcome}, and {@code final_t_s}.
 */
public final class LaunchTofTable {

  private static final double SIM_DT = 0.001;
  private static final double HIT_TOLERANCE_M = 0.40;
  private static final int MAX_STEPS = 20000;
  /** Record ball position every N sim steps while seeding (1 step = 1 ms). */
  private static final int SEED_TRAJECTORY_LOG_EVERY_N_STEPS = 25;

  private static final Alert tofTableSimFailedAlert =
      new Alert(
          "Launch ToF lookup table: one or more distances were skipped (simulation failed).",
          AlertType.kError);

  private LaunchTofTable() {}

  private static final double[] SCORING_DISTANCES_M = {1.03, 2.1, 3.36, 5.0, 6.03};

  private static final double[] AZ_EXTRA_DISTANCES_M = {2.11, 6.44};

  public static void seedScoringTofMap(
      InterpolatingDoubleTreeMap dest,
      InterpolatingDoubleTreeMap rpmMap,
      InterpolatingDoubleTreeMap hoodMap,
      double muzzleHeightM) {
    for (double d : SCORING_DISTANCES_M) {
      tryPutTofEntry(dest, d, rpmMap, hoodMap, muzzleHeightM);
    }
  }

  public static void seedAllianceZoneTofMap(
      InterpolatingDoubleTreeMap dest,
      InterpolatingDoubleTreeMap rpmMap,
      InterpolatingDoubleTreeMap hoodMap,
      double muzzleHeightM) {
    for (double d : SCORING_DISTANCES_M) {
      tryPutTofEntry(dest, d, rpmMap, hoodMap, muzzleHeightM);
    }
    for (double d : AZ_EXTRA_DISTANCES_M) {
      tryPutTofEntry(dest, d, rpmMap, hoodMap, muzzleHeightM);
    }
  }

  /** Runs drag-aware simulation; on success inserts into {@code dest}; on failure logs only. */
  private static void tryPutTofEntry(
      InterpolatingDoubleTreeMap dest,
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
      logSeedTrajectorySkippedGeometry(horizontalDistanceM);
      failTofEntry(
          horizontalDistanceM,
          "invalid launch geometry (cos(theta)<0.05 or muzzle speed<1e-3 m/s)");
      return;
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

    var sampleT = new ArrayList<Double>();
    var sampleX = new ArrayList<Double>();
    var sampleY = new ArrayList<Double>();
    var sampleZ = new ArrayList<Double>();
    appendTrajectorySample(sampleT, sampleX, sampleY, sampleZ, 0, start);

    double t = 0;
    for (int i = 0; i < MAX_STEPS; i++) {
      ball.update(Seconds.of(SIM_DT));
      t += SIM_DT;
      Translation3d p = ball.getPosition();
      boolean hit = p.getDistance(target) < HIT_TOLERANCE_M;
      boolean timeout = t > 10.0 || p.getZ() < 0;
      if ((i + 1) % SEED_TRAJECTORY_LOG_EVERY_N_STEPS == 0 || hit || timeout) {
        appendTrajectorySample(sampleT, sampleX, sampleY, sampleZ, t, p);
      }
      if (hit) {
        dest.put(horizontalDistanceM, t);
        logSeedTrajectory(
            horizontalDistanceM, target, sampleT, sampleX, sampleY, sampleZ, "hit", t);
        return;
      }
      if (timeout) {
        logSeedTrajectory(
            horizontalDistanceM, target, sampleT, sampleX, sampleY, sampleZ, "miss", t);
        failTofEntry(
            horizontalDistanceM,
            "trajectory did not reach hub within tolerance (timeout, ground impact, or miss)");
        return;
      }
    }
    Translation3d p = ball.getPosition();
    appendTrajectorySample(sampleT, sampleX, sampleY, sampleZ, t, p);
    logSeedTrajectory(horizontalDistanceM, target, sampleT, sampleX, sampleY, sampleZ, "miss", t);
    failTofEntry(
        horizontalDistanceM,
        "trajectory did not reach hub within tolerance (timeout, ground impact, or miss)");
  }

  private static String seedLogPath(double horizontalDistanceM) {
    return "LaunchTofTable/seed/"
        + String.format(Locale.US, "d_%s", Double.toString(horizontalDistanceM)).replace('.', 'p');
  }

  private static void appendTrajectorySample(
      ArrayList<Double> sampleT,
      ArrayList<Double> sampleX,
      ArrayList<Double> sampleY,
      ArrayList<Double> sampleZ,
      double tSeconds,
      Translation3d p) {
    sampleT.add(tSeconds);
    sampleX.add(p.getX());
    sampleY.add(p.getY());
    sampleZ.add(p.getZ());
  }

  private static void logSeedTrajectory(
      double horizontalDistanceM,
      Translation3d target,
      ArrayList<Double> sampleT,
      ArrayList<Double> sampleX,
      ArrayList<Double> sampleY,
      ArrayList<Double> sampleZ,
      String outcome,
      double finalTS) {
    String base = seedLogPath(horizontalDistanceM);
    Logger.recordOutput(base + "/target", target);
    Logger.recordOutput(base + "/outcome", outcome);
    Logger.recordOutput(base + "/final_t_s", finalTS);
    Logger.recordOutput(base + "/ball_t_s", toPrimitiveArray(sampleT));
    Logger.recordOutput(base + "/ball_x_m", toPrimitiveArray(sampleX));
    Logger.recordOutput(base + "/ball_y_m", toPrimitiveArray(sampleY));
    Logger.recordOutput(base + "/ball_z_m", toPrimitiveArray(sampleZ));
  }

  private static void logSeedTrajectorySkippedGeometry(double horizontalDistanceM) {
    String base = seedLogPath(horizontalDistanceM);
    Logger.recordOutput(base + "/outcome", "skipped_geometry");
    Logger.recordOutput(base + "/ball_t_s", new double[0]);
    Logger.recordOutput(base + "/ball_x_m", new double[0]);
    Logger.recordOutput(base + "/ball_y_m", new double[0]);
    Logger.recordOutput(base + "/ball_z_m", new double[0]);
  }

  private static double[] toPrimitiveArray(ArrayList<Double> list) {
    double[] a = new double[list.size()];
    for (int i = 0; i < list.size(); i++) {
      a[i] = list.get(i);
    }
    return a;
  }

  private static void failTofEntry(double horizontalDistanceM, String detail) {
    tofTableSimFailedAlert.set(true);
    DriverStation.reportWarning(
        "LaunchTofTable: skipped distance " + horizontalDistanceM + " m — " + detail, false);
  }
}
