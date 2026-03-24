package frc2713.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc2713.lib.util.BidirectionalInterpolatingDoubleMap;
import frc2713.lib.util.LoggedTunableNumber;

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
  public static InterpolatingDoubleTreeMap rpmToReleaseAngleAdjustmentMap =
      new InterpolatingDoubleTreeMap();

  /** Distance to hub (m) -> Time of flight (s) */
  public static InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();
  /** Distance to az corner (m) -> Time of flight (s) */
  public static InterpolatingDoubleTreeMap tofMapAZ = new InterpolatingDoubleTreeMap();
  /** Muzzle height above carpet used when generating drag-aware ToF lookup tables. */
  public static final double tofSeedMuzzleHeightMeters = Inches.of(22).in(Meters);

  static {
    // Distance to hub center (m) -> RPM (rpm)
    distanceToRpmMap.put(1.03, 2500.);
    distanceToRpmMap.put(2.1, 2500.);
    distanceToRpmMap.put(3.36, 3500.);
    distanceToRpmMap.put(5.0, 4000.);
    distanceToRpmMap.put(6.03, 5000.);

    // Distance to az Corner (m) -> RPM (rpm)
    distanceToRpmAzMap.put(1.03, 2400.);
    distanceToRpmAzMap.put(1.75, 2666.66);
    distanceToRpmAzMap.put(2.1, 2800.);
    distanceToRpmAzMap.put(3.36, 2933.);
    distanceToRpmAzMap.put(4.5, 3066.);
    distanceToRpmAzMap.put(5.0, 3333.);
    distanceToRpmAzMap.put(6.03, 4266.);

    // Ball Velocity (ft/s) <-> RPM (rpm)

    velocityToRpmBiDiMap.put(17.69, 2500.);
    velocityToRpmBiDiMap.put(20.19, 3000.);
    velocityToRpmBiDiMap.put(22.07, 3500.);
    velocityToRpmBiDiMap.put(24.67, 4000.);
    velocityToRpmBiDiMap.put(25.52, 4500.);

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
    rpmToReleaseAngleAdjustmentMap.put(2000., 5.0);
    rpmToReleaseAngleAdjustmentMap.put(3000., 5.);
    rpmToReleaseAngleAdjustmentMap.put(3500., 10.);
    rpmToReleaseAngleAdjustmentMap.put(4000., 12.5);
    rpmToReleaseAngleAdjustmentMap.put(4500., 15.);
    rpmToReleaseAngleAdjustmentMap.put(5000., 15.);

    /** Distance to hub (m) -> Time of flight (s) */
    tofMap.put(0.974, 1.14);
    tofMap.put(1.443, 1.18);
    tofMap.put(2.905, 1.2);
    tofMap.put(3.345, 1.2);
    tofMap.put(3.945, 1.22);
    tofMap.put(5.422, 1.38);
    tofMap.put(6.329, 1.52);

    tofMapAZ.put(0.974, 1.14);
    tofMapAZ.put(1.443, 1.18);
    tofMapAZ.put(2.905, 1.2);
    tofMapAZ.put(3.345, 1.2);
    tofMapAZ.put(3.945, 1.22);
    tofMapAZ.put(5.422, 1.38);
    tofMapAZ.put(6.329, 1.52);
  }

  public static AngularVelocity getVelocityFromDistance(Distance dist, boolean isHub) {
    if (isHub) {
      return RPM.of(distanceToRpmMap.get(dist.in(Meters)));
    }
    return RPM.of(distanceToRpmAzMap.get(dist.in(Meters)));
  }

  public static AngularVelocity getFlywheelVelocityFromBallVelocity(LinearVelocity ballVelocity) {
    return RPM.of(velocityToRpmBiDiMap.get(ballVelocity.in(FeetPerSecond)));
  }

  public static LinearVelocity getBallVelocityFromFlywheelVelocity(
      AngularVelocity flywheelVelocity) {
    return FeetPerSecond.of(velocityToRpmBiDiMap.reverseGet(flywheelVelocity.in(RPM)));
  }

  public static Angle getHoodAngleFromDistance(Distance horizontalDistance, boolean isHub) {
    if (isHub) {
      return Degrees.of(distanceToAngleMap.get(horizontalDistance.in(Meters)));
    }
    return Degrees.of(distanceToAngleAzMap.get(horizontalDistance.in(Meters)));
  }

  public static Angle getReleaseAngleAdjustmentFromFlywheelVelocity(
      AngularVelocity flywheelVelocity) {
    return Degrees.of(rpmToReleaseAngleAdjustmentMap.get(flywheelVelocity.in(RPM)));
  }

  public static Angle getReleaseAngleFromDistanceAndFlywheelVelocity(
      Distance horizontalDistance, AngularVelocity flywheelVelocity, boolean isHub) {
    return getHoodAngleFromDistance(horizontalDistance, isHub)
        .plus(getReleaseAngleAdjustmentFromFlywheelVelocity(flywheelVelocity));
  }

  public static Time getTimeOfFlight(Distance horizontalDistance, boolean isHub) {
    if (isHub) {
      return Seconds.of(tofMap.get(horizontalDistance.in(Meters)));
    }
    return Seconds.of(tofMapAZ.get(horizontalDistance.in(Meters)));
  }

  public static LoggedTunableNumber tuningMagnus = new LoggedTunableNumber("Tuning Magnus", 0.5);
}
