package frc2713.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc2713.lib.util.BidirectionalInterpolatingDoubleMap;
import frc2713.lib.util.LoggedTunableNumber;
import frc2713.robot.subsystems.launcher.LauncherConstants.Flywheels;
import frc2713.robot.util.LaunchTofTable;

public final class LaunchingLookupMaps {

  // TODO: see if feed shot maps could be replaced with constant offsets

  /** Distance to hub (m) -> RPM (rpm) */
  public static InterpolatingDoubleTreeMap distanceToRpmMap = new InterpolatingDoubleTreeMap();
  /** Distance to az corner (m) -> RPM (rpm) */
  public static InterpolatingDoubleTreeMap distanceToRpmAzMap = new InterpolatingDoubleTreeMap();

  /** Distance to hub (m) -> Angle (deg) */
  public static InterpolatingDoubleTreeMap distanceToAngleMap = new InterpolatingDoubleTreeMap();
  /** Distance to az corner (m) -> Angle (deg) */
  public static InterpolatingDoubleTreeMap distanceToAngleAzMap = new InterpolatingDoubleTreeMap();

  /** Ball velocity (m/s) <-> RPM (rpm) */
  public static BidirectionalInterpolatingDoubleMap velocityToRpmBiDiMap =
      new BidirectionalInterpolatingDoubleMap();

  /** Hood Angle (deg) -> Release Angle (deg) */
  public static InterpolatingDoubleTreeMap hoodAngleToReleaseAngleMap =
      new InterpolatingDoubleTreeMap();

  /** Distance to hub (m) -> Time of flight (s) */
  public static InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();
  /** Distance to az corner (m) -> Time of flight (s) */
  public static InterpolatingDoubleTreeMap tofMapAZ = new InterpolatingDoubleTreeMap();
  /** Muzzle height above carpet used when generating drag-aware ToF lookup tables. */
  public static final double tofSeedMuzzleHeightMeters = Inches.of(22).in(Meters);

  static {
    // Distance to hub center (m) -> RPM (rpm)
    distanceToRpmMap.put(1.03, 1800.);
    distanceToRpmMap.put(1.75, 2000.);
    distanceToRpmMap.put(2.1, 2100.);
    distanceToRpmMap.put(3.36, 2200.);
    distanceToRpmMap.put(4.5, 2300.);
    distanceToRpmMap.put(5.0, 2500.);
    distanceToRpmMap.put(6.03, 3200.);

    // Distance to az Corner (m) -> RPM (rpm)
    distanceToRpmAzMap.put(1.03, 1800.);
    distanceToRpmAzMap.put(1.75, 2000.);
    distanceToRpmAzMap.put(2.1, 2100.);
    distanceToRpmAzMap.put(3.36, 2200.);
    distanceToRpmAzMap.put(4.5, 2300.);
    distanceToRpmAzMap.put(5.0, 2500.);

    // Ball Velocity (ft/s) <-> RPM (rpm)
    velocityToRpmBiDiMap.put(16.0, 1800.);
    velocityToRpmBiDiMap.put(19.2, 2000.);
    velocityToRpmBiDiMap.put(25.7, 3000.);
    velocityToRpmBiDiMap.put(28.9, 3500.);
    velocityToRpmBiDiMap.put(30.3, 4000.);
    velocityToRpmBiDiMap.put(35.3, 4500.);

    // Distance to hub center (m) -> Hood Pitch (Degrees)
    distanceToAngleMap.put(1.03, 5.0);
    distanceToAngleMap.put(2.1, 20.0);
    distanceToAngleMap.put(3.36, 25.0);
    distanceToAngleMap.put(5.0, 27.13);
    distanceToAngleMap.put(6.03, 30.0);

    // Distance to AZ corner (m) -> Hood Pitch (Degrees)
    distanceToAngleAzMap.put(1.03, 5.0);
    distanceToAngleAzMap.put(2.1, 20.0);
    distanceToAngleAzMap.put(3.36, 25.0);
    distanceToAngleAzMap.put(5.0, 27.13);
    distanceToAngleAzMap.put(6.03, 30.0);

    // Hood angle (deg) -> release angle (deg)
    hoodAngleToReleaseAngleMap.put(0., 15.);
    hoodAngleToReleaseAngleMap.put(15., 35.);
    hoodAngleToReleaseAngleMap.put(30., 50.);

    LaunchTofTable.seedScoringTofMap(
        tofMap, distanceToRpmMap, distanceToAngleMap, tofSeedMuzzleHeightMeters);
    LaunchTofTable.seedAllianceZoneTofMap(
        tofMapAZ, distanceToRpmAzMap, distanceToAngleAzMap, tofSeedMuzzleHeightMeters);
  }

  public static double exitAngleRadiansFromHoodDegrees(double degrees) {
    return Degrees.of(hoodAngleToReleaseAngleMap.get(degrees)).in(Radians);
  }

  /**
   * Nominal muzzle speed (m/s) from flywheel mechanism RPM using {@link Flywheels#WHEEL_DIAMETER}
   * as contact radius and {@link #rpmToMuzzleVelocityScale}.
   */
  public static double muzzleVelocityMetersPerSecond(double flywheelRpm) {
    return velocityToRpmBiDiMap.reverseGet(flywheelRpm);
  }

  public static LoggedTunableNumber tuningMagnus = new LoggedTunableNumber("Tuning Magnus", 0.5);
}
