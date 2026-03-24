package frc2713.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import frc2713.lib.geometry.GeometryUtil;
import frc2713.lib.util.LoggedTunableBoolean;
import frc2713.lib.util.LoggedTunableNumber;
import frc2713.robot.Constants;
import frc2713.robot.FieldConstants;
import frc2713.robot.RobotContainer;
import java.util.LinkedHashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class VisionIOSLAMDunk implements VisionIO {
  private static final int STD_DEV_CACHE_SIZE = 256;
  private static final double AVG_TAG_SIZE_BIN_PERCENT = 1.0;
  private static final Distance BASE_TRANSLATION_STD_DEV =
      VisionConstants.POSE_ESTIMATOR_STATE_STDEVS.translationalStDev();
  private static final double ROUGH_DIST_COEFFICIENT = 74.7;
  private static final double ROUGH_DIST_EXPONENT = -0.396;
  private static final double ROTATION_STD_DEV_DEGREES = 999.0;

  private NetworkTableInstance inst;
  private NetworkTable table;
  private DoubleArraySubscriber sub;
  private double lastTimestamp = -1;
  private static final LoggedTunableBoolean useStdDevCache =
      new LoggedTunableBoolean("Vision/useStdDevCache", true);
  private static final LoggedTunableNumber k = new LoggedTunableNumber("Vision/k", 2);
  private static final Transform3d SLAMDUNK_TRANSFORM =
      new Transform3d(new Translation3d(), new Rotation3d(0, 0, Math.PI / 2));
  private final Map<StdDevCacheKey, StdDevCacheEntry> stdDevCache =
      new LinkedHashMap<>(STD_DEV_CACHE_SIZE, 0.75f, true) {
        @Override
        protected boolean removeEldestEntry(Map.Entry<StdDevCacheKey, StdDevCacheEntry> eldest) {
          return size() > STD_DEV_CACHE_SIZE;
        }
      };
  private long stdDevCacheHits = 0;
  private long stdDevCacheMisses = 0;
  private Boolean lastStdDevCacheEnabled = null;

  public VisionIOSLAMDunk() {
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("slamdunk");
    sub = table.getDoubleArrayTopic("pose_robot").subscribe(new double[0]);
  }

  @Override
  public void updateInputs(VisionInputs inputs) {
    // inputs.translationStdDev = VisionConstants.POSE_ESTIMATOR_STATE_STDEVS.translationalStDev();
    // inputs.rotationStdDev = VisionConstants.POSE_ESTIMATOR_STATE_STDEVS.rotationalStDev();

    // Reset pose to zero, leave pose3d as last state for visualization
    inputs.pose = GeometryUtil.Constants.Pose_2d.ZERO;

    var poseArray = sub.get();

    if (poseArray.length <= 10) {
      inputs.reasoning = "No pose data available";
      inputs.applying = false;
      return;
    }

    double t = poseArray[0];

    if (lastTimestamp == t) {
      inputs.reasoning = "Stale timestamp";
      inputs.applying = false;
      return;
    }

    if (Constants.tuningMode && RobotContainer.drive != null) {
      var linspeed = RobotContainer.drive.getSpeed();
      var angspeed = RobotContainer.drive.getAngularSpeed();
      Logger.recordOutput("SLAMDunk/SpeedLinear", linspeed);
      Logger.recordOutput("SLAMDunk/SpeedAngular", angspeed);
      Logger.recordOutput("SLAMDunk/tFPGA", Timer.getFPGATimestamp());
    }

    inputs.tagCount = (int) poseArray[8];
    inputs.subgraphError = poseArray[9];
    inputs.avgTagSize = poseArray[10];
    inputs.timestamp = t;
    double latency = Timer.getFPGATimestamp() - t;
    inputs.latency = Seconds.of(latency);

    lastTimestamp = t;
    var rawPose =
        new Pose3d(
            new Translation3d(poseArray[1], poseArray[2], poseArray[3]),
            new Rotation3d(new Quaternion(poseArray[4], poseArray[5], poseArray[6], poseArray[7])));

    inputs.rawYaw = Rotation2d.fromRadians(rawPose.getRotation().getZ());

    inputs.pose3d = rawPose.transformBy(SLAMDUNK_TRANSFORM);

    inputs.pose = inputs.pose3d.toPose2d();

    if (latency < 0) {
      inputs.reasoning = "Pose from the future!";
      inputs.applying = false;
      return;
    }
    if (inputs.latency.compareTo(VisionConstants.LATENCY_THRESHOLD.get()) > 0) {
      inputs.reasoning = "Latency above threshold";
      inputs.applying = false;
      return;
    }

    if (RobotContainer.drive != null) {
      if (!FieldConstants.FIELD_PLUS_HALF_METER.contains(
          RobotContainer.drive.getPose().getTranslation())) {
        inputs.reasoning = "ROBOT OUTSIDE FIELD!! HARD RESET";
        inputs.applying = true;
        RobotContainer.drive.setPose(inputs.pose);
        Logger.recordOutput("Vision/robotOutsideField", true);
        return;
      } else {
        Logger.recordOutput("Vision/robotOutsideField", false);
      }
    }

    if (!FieldConstants.FIELD_PLUS_HALF_METER.contains(inputs.pose.getTranslation())) {
      inputs.reasoning = "Vision outside field";
      inputs.applying = false;
      return;
    }

    if (Math.abs(inputs.pose3d.getTranslation().getZ()) > 0.1) {
      inputs.applying = false;
      inputs.reasoning = "Off ground Z > 0.1m";
      return;
    }

    boolean stdDevCacheEnabled = useStdDevCache.get();
    if (lastStdDevCacheEnabled == null || lastStdDevCacheEnabled != stdDevCacheEnabled) {
      stdDevCache.clear();
      stdDevCacheHits = 0;
      stdDevCacheMisses = 0;
      lastStdDevCacheEnabled = stdDevCacheEnabled;
    }

    double stdDevExponent = k.get();
    StdDevCacheEntry stdDevEntry;
    if (stdDevCacheEnabled) {
      stdDevEntry = getStdDevCacheEntry(inputs.avgTagSize, inputs.tagCount, stdDevExponent);
    } else {
      stdDevEntry = calculateStdDevEntry(inputs.avgTagSize, inputs.tagCount, stdDevExponent);
    }
    Logger.recordOutput("Vision/roughDist", stdDevEntry.roughDist());
    Logger.recordOutput("Vision/distanceScaleFactor", stdDevEntry.distScaleFactor());
    Logger.recordOutput("Vision/countScaleFactor", stdDevEntry.countScaleFactor());
    if (Constants.tuningMode) {
      long accesses = stdDevCacheHits + stdDevCacheMisses;
      double hitRate = accesses > 0 ? (double) stdDevCacheHits / accesses : 0.0;
      Logger.recordOutput("Vision/stdDevCacheEnabled", stdDevCacheEnabled);
      Logger.recordOutput("Vision/stdDevCacheHits", (double) stdDevCacheHits);
      Logger.recordOutput("Vision/stdDevCacheMisses", (double) stdDevCacheMisses);
      Logger.recordOutput("Vision/stdDevCacheHitRate", hitRate);
      Logger.recordOutput("Vision/stdDevCacheSize", (double) stdDevCache.size());
    }

    inputs.translationStdDev = stdDevEntry.translationStdDev();
    inputs.rotationStdDev = Degrees.of(ROTATION_STD_DEV_DEGREES);
    inputs.reasoning = "Valid pose.";
    inputs.applying = true;
    return;
  }

  private StdDevCacheEntry getStdDevCacheEntry(
      double avgTagSize, int tagCount, double stdDevExponent) {
    double quantizedAvgTagSize = quantizeAvgTagSize(avgTagSize);
    StdDevCacheKey key = StdDevCacheKey.from(quantizedAvgTagSize, tagCount, stdDevExponent);
    StdDevCacheEntry cachedEntry = stdDevCache.get(key);
    if (cachedEntry != null) {
      stdDevCacheHits++;
      return cachedEntry;
    }
    stdDevCacheMisses++;

    StdDevCacheEntry newEntry = calculateStdDevEntry(quantizedAvgTagSize, tagCount, stdDevExponent);
    stdDevCache.put(key, newEntry);
    return newEntry;
  }

  private StdDevCacheEntry calculateStdDevEntry(
      double avgTagSize, int tagCount, double stdDevExponent) {
    double quantizedAvgTagSize = quantizeAvgTagSize(avgTagSize);
    double roughDist = ROUGH_DIST_COEFFICIENT * Math.pow(quantizedAvgTagSize, ROUGH_DIST_EXPONENT);
    double distScaleFactor = Math.pow(roughDist, stdDevExponent);
    int tagCountSquared = tagCount * tagCount;
    double countScaleFactor = 1.0 / Math.max(1, tagCountSquared);

    return new StdDevCacheEntry(
        BASE_TRANSLATION_STD_DEV.times(distScaleFactor).times(countScaleFactor),
        roughDist,
        distScaleFactor,
        countScaleFactor);
  }

  private static double quantizeAvgTagSize(double avgTagSize) {
    if (avgTagSize <= 0.0) {
      return 0.0;
    }
    // 1% bins: round to nearest 0.01 in avgTagSize units.
    double binsPerUnit = 100.0 / AVG_TAG_SIZE_BIN_PERCENT;
    return Math.round(avgTagSize * binsPerUnit) / binsPerUnit;
  }

  private record StdDevCacheKey(long avgTagSizeBits, int tagCount, long exponentBits) {
    static StdDevCacheKey from(double avgTagSize, int tagCount, double stdDevExponent) {
      return new StdDevCacheKey(
          Double.doubleToLongBits(avgTagSize), tagCount, Double.doubleToLongBits(stdDevExponent));
    }
  }

  private record StdDevCacheEntry(
      Distance translationStdDev,
      double roughDist,
      double distScaleFactor,
      double countScaleFactor) {}
}
