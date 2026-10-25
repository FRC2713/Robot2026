package org.spectrum3847.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotState;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.spectrum3847.vision.compat.Clock;
import org.spectrum3847.vision.gate.GateContext;
import org.spectrum3847.vision.gate.GateSet;
import org.spectrum3847.vision.gate.Gates;
import org.spectrum3847.vision.health.VisionHealth;
import org.spectrum3847.vision.heading.RobotMotionHistory;
import org.spectrum3847.vision.io.CameraIO;
import org.spectrum3847.vision.io.PhotonCameraIO;
import org.spectrum3847.vision.log.NetworkTablesVisionLogger;
import org.spectrum3847.vision.log.VisionLogger;
import org.spectrum3847.vision.log.VisionStats;
import org.spectrum3847.vision.observation.FrameObservation;
import org.spectrum3847.vision.sink.PoseSink;
import org.spectrum3847.vision.solve.PoseSolver;
import org.spectrum3847.vision.solve.Solvers;
import org.spectrum3847.vision.trust.TrustModel;

/**
 * Every camera, every frame, into the pose estimator: solved, gated, given a trust, and sent in
 * capture-time order. One object, one call per loop.
 *
 * <pre>{@code
 * vision = VisionSystem.builder(AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField))
 *     .camera("TopLeft", robotToTopLeft)
 *     .camera("TopRight", robotToTopRight)
 *     .sink(PoseSink.wpilib(poseEstimator))
 *     .currentEstimate(poseEstimator::getEstimatedPosition)
 *     .build();
 *
 * // robotPeriodic(), after odometry:
 * vision.addMotion(Timer.getTimestamp(), gyroYaw, yawRateRadPerSec, speedMetersPerSec);
 * vision.periodic();
 * }</pre>
 *
 * See robot-vision/README.md for CTRE swerve, AdvantageKit and tuning.
 */
public class VisionSystem {
    private final AprilTagFieldLayout field;
    private final List<VisionCamera> cameras;
    private final Map<String, VisionCamera> byName = new LinkedHashMap<>();
    private final PoseSolver solver;
    private final GateSet.Driver gates;
    private final TrustModel trust;
    private final PoseSink sink;
    private final Supplier<Pose2d> currentEstimate;
    private final BooleanSupplier enabled;
    private final RobotMotionHistory motion;
    private final MotionSource motionSource;
    private final Set<Integer> excluded;
    private final IntegerArraySubscriber jetsonExcluded;
    private final VisionLogger logger;
    private final VisionHealth health;
    private final VisionStats stats = new VisionStats();
    private final DoubleSupplier clock;
    private VisionUpdate last;

    /** Polled each {@link #periodic()} when given: the heading, turn rate and speed now. */
    @FunctionalInterface
    public interface MotionSource {
        /** @return {heading radians, yaw rate rad/s, speed m/s} */
        double[] sample();
    }

    private VisionSystem(Builder b) {
        field = b.field;
        clock = b.clock;
        cameras = List.copyOf(b.cameras);
        for (var c : cameras) byName.put(c.name(), c);
        solver = b.solver;
        gates = new GateSet.Driver(b.gates);
        trust = b.trust;
        sink = b.sink;
        currentEstimate = b.currentEstimate;
        enabled = b.enabled;
        motion = b.motion;
        motionSource = b.motionSource;
        excluded = Set.copyOf(b.excluded);
        jetsonExcluded = b.readJetsonExcludedTags
                ? NetworkTableInstance.getDefault()
                        .getTable("photonvision")
                        .getIntegerArrayTopic("excludedTagsActive")
                        .subscribe(new long[0])
                : null;
        logger = b.logger != null ? b.logger : new NetworkTablesVisionLogger("SpectrumVision", field);
        health = b.health ? new VisionHealth(clock.getAsDouble()) : null;
    }

    public static Builder builder(AprilTagFieldLayout field) {
        return new Builder(field);
    }

    /** The robot's heading, turn rate and speed at {@code timestampSeconds} (robot time). Call every loop. */
    public void addMotion(double timestampSeconds, Rotation2d heading, double yawRateRadPerSec, double speedMetersPerSec) {
        motion.add(timestampSeconds, heading, yawRateRadPerSec, speedMetersPerSec);
    }

    /** One robot loop: read every camera, solve, gate, trust, send. Returns what happened. */
    public VisionUpdate periodic() {
        double now = clock.getAsDouble();
        if (motionSource != null) {
            double[] m = motionSource.sample();
            motion.add(now, new Rotation2d(m[0]), m[1], m[2]);
        }
        var ctx = new GateContext(
                now,
                field,
                motion,
                currentEstimate == null ? null : currentEstimate.get(),
                enabled.getAsBoolean(),
                excludedTags());
        gates.beginLoop(ctx);

        var frames = new ArrayList<FrameObservation>();
        var accepted = new ArrayList<VisionUpdate.Accepted>();
        var rejected = new ArrayList<VisionUpdate.Rejected>();
        var inputs = new LinkedHashMap<String, CameraIO.Inputs>();
        for (var cam : cameras) {
            cam.io().update(cam.inputs);
            inputs.put(cam.name(), cam.inputs);
            if (!cam.enabled()) continue;
            var robotToCamera = cam.robotToCamera();
            for (var frame : cam.inputs.frames) {
                frames.add(frame);
                for (var obs : solver.solve(frame, robotToCamera, field, motion)) {
                    var v = gates.judge(obs, ctx);
                    if (v.accepted()) {
                        accepted.add(new VisionUpdate.Accepted(obs, trust.stdDevs(obs, ctx, cam.trustFactor())));
                    } else {
                        rejected.add(new VisionUpdate.Rejected(obs, v.gate(), v.reason()));
                    }
                }
            }
        }
        // Oldest first: WPILib's estimator drops a measurement older than one already applied.
        accepted.sort(Comparator.comparingDouble(a -> a.observation().timestampSeconds()));
        for (var a : accepted) {
            sink.addVisionMeasurement(a.pose(), a.observation().timestampSeconds(), a.stdDevs());
            gates.accepted(a.observation(), ctx);
        }
        last = new VisionUpdate(now, frames, accepted, rejected);
        stats.record(last);
        logger.log(last, stats);
        if (health != null) {
            var loop = new VisionStats.Counts();
            loop.candidates = accepted.size() + rejected.size();
            loop.accepted = accepted.size();
            health.update(now, ctx.enabled(), inputs, loop, topReason(rejected));
        }
        return last;
    }

    private static String topReason(List<VisionUpdate.Rejected> rejected) {
        var counts = new TreeMap<String, Integer>();
        for (var r : rejected) counts.merge(r.gate(), 1, Integer::sum);
        return counts.entrySet().stream().max(Map.Entry.comparingByValue()).map(Map.Entry::getKey).orElse("none");
    }

    /** Your excluded tags plus the Jetson's ({@code /photonvision/excludedTagsActive}). */
    public Set<Integer> excludedTags() {
        if (jetsonExcluded == null) return excluded;
        long[] j = jetsonExcluded.get();
        if (j.length == 0) return excluded;
        var all = new HashSet<>(excluded);
        for (long id : j) all.add((int) id);
        return all;
    }

    /** The last loop's result (null before the first). */
    public VisionUpdate lastUpdate() {
        return last;
    }

    public VisionStats stats() {
        return stats;
    }

    /** The dashboard alerts' settings (thresholds are public fields); null if health is off. */
    public VisionHealth health() {
        return health;
    }

    public List<VisionCamera> cameras() {
        return cameras;
    }

    public VisionCamera camera(String name) {
        return byName.get(name);
    }

    public RobotMotionHistory motion() {
        return motion;
    }

    public AprilTagFieldLayout field() {
        return field;
    }

    public static final class Builder {
        private final AprilTagFieldLayout field;
        private final List<VisionCamera> cameras = new ArrayList<>();
        private PoseSolver solver = Solvers.standard();
        private GateSet gates = Gates.recommended();
        private TrustModel trust = TrustModel.recommended();
        private PoseSink sink;
        private Supplier<Pose2d> currentEstimate;
        private BooleanSupplier enabled = RobotState::isEnabled;
        private RobotMotionHistory motion = new RobotMotionHistory();
        private MotionSource motionSource;
        private final Set<Integer> excluded = new HashSet<>();
        private boolean readJetsonExcludedTags = true;
        private VisionLogger logger;
        private boolean health = true;
        private DoubleSupplier clock = Clock::now;

        private Builder(AprilTagFieldLayout field) {
            this.field = field;
        }

        /** A PhotonVision camera, by its name in PhotonVision, at {@code robotToCamera}. */
        public Builder camera(String name, Transform3d robotToCamera) {
            return camera(PhotonCameraIO.of(name), robotToCamera, 1.0);
        }

        /** Any camera source (a FakeCameraIO in tests, a replay), with its own trust factor. */
        public Builder camera(CameraIO io, Transform3d robotToCamera, double trustFactor) {
            cameras.add(new VisionCamera(io, robotToCamera, trustFactor));
            return this;
        }

        public Builder solver(PoseSolver s) {
            solver = s;
            return this;
        }

        public Builder gates(GateSet g) {
            gates = g;
            return this;
        }

        public Builder trust(TrustModel t) {
            trust = t;
            return this;
        }

        /** Required: where accepted measurements go (PoseSink.wpilib, a CTRE drivetrain, ...). */
        public Builder sink(PoseSink s) {
            sink = s;
            return this;
        }

        /** The estimator's current pose, for the innovation gate (recommended). */
        public Builder currentEstimate(Supplier<Pose2d> s) {
            currentEstimate = s;
            return this;
        }

        /** Whether the robot is enabled (default RobotState.isEnabled). */
        public Builder enabled(BooleanSupplier s) {
            enabled = s;
            return this;
        }

        /** Instead of calling addMotion yourself: polled at the start of each periodic(). */
        public Builder motionSource(MotionSource s) {
            motionSource = s;
            return this;
        }

        public Builder motionHistory(RobotMotionHistory h) {
            motion = h;
            return this;
        }

        /** Tags never to use (on top of the Jetson's list). */
        public Builder excludeTags(int... ids) {
            for (int id : ids) excluded.add(id);
            return this;
        }

        /** Also read the Jetson's excluded tags from NetworkTables (default on). */
        public Builder readJetsonExcludedTags(boolean on) {
            readJetsonExcludedTags = on;
            return this;
        }

        /** Default: NetworkTablesVisionLogger under /SpectrumVision. VisionLogger.NONE for none. */
        public Builder logger(VisionLogger l) {
            logger = l;
            return this;
        }

        /** Dashboard alerts (default on). */
        public Builder health(boolean on) {
            health = on;
            return this;
        }

        /** Robot time in seconds (default Timer.getTimestamp()); tests give their own. */
        public Builder clock(DoubleSupplier c) {
            clock = c;
            return this;
        }

        /**
         * For unit tests and replays off the robot: no NetworkTables (no logger, no alerts, no
         * Jetson excluded tags), robot enabled. Set the clock too.
         */
        public Builder offline() {
            logger = VisionLogger.NONE;
            health = false;
            readJetsonExcludedTags = false;
            enabled = () -> true;
            return this;
        }

        public VisionSystem build() {
            if (sink == null) throw new IllegalStateException("VisionSystem needs a sink: .sink(PoseSink.wpilib(estimator))");
            if (cameras.isEmpty()) throw new IllegalStateException("VisionSystem has no cameras");
            return new VisionSystem(this);
        }
    }
}
