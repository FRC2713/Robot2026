package org.spectrum3847.vision.gate;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.ArrayDeque;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;
import org.spectrum3847.vision.observation.PoseObservation;
import org.spectrum3847.vision.observation.PoseObservation.Type;

/**
 * The built-in gates. {@link #recommended()} is the set we run; build your own with {@link
 * GateSet#of} from these and {@link Gate#of}. Every limit is a plain number you can change, and
 * every rejection is counted by gate name, so a tuning change shows up in the logs as "rejected by
 * maxDistance: 41 → 3".
 *
 * <p>Why these, and the defaults, are explained in robot-vision/README.md ("Gates").
 */
public final class Gates {
    private Gates() {}

    /**
     * What we recommend, in this order (cheap and certain first):
     *
     * <ol>
     *   <li>{@link #uniqueFrames()}: the same frame never counts twice
     *   <li>{@link #timeSynced(double)} 0.5 s: PhotonVision's clock is synced to the robot's
     *   <li>{@link #maxAge(double)} 0.5 s and {@link #notFromFuture(double)} 0.05 s
     *   <li>{@link #excludedTags()}: no excluded tag behind it
     *   <li>{@link #insideField(double)} 0.5 m margin
     *   <li>{@link #maxHeight(double)} 0.3 m and {@link #maxTilt(double)} 20°: robots stay on the
     *       floor (20° leaves room for a field's bumps or ramps)
     *   <li>{@link #maxSingleTagAmbiguity(double)} 0.3
     *   <li>{@link #maxDistance(double, double)} 4.5 m one tag, 7 m multi-tag
     *   <li>{@link #maxReprojection(double)} 3 px
     *   <li>{@link #maxYawRate(double)} 3.5 rad/s: a spinning robot's frames are blurred and its
     *       timestamp error costs the most
     *   <li>{@link #headingAgreesWithGyro(double)} 15°: a full-pose solve far from the gyro is a bad
     *       solve, a wrong tag map, or a wrong camera mount
     *   <li>{@link #innovation(InnovationConfig)}: no sudden jump away from the estimate, unless
     *       several frames in a row agree on it (or the robot is disabled)
     * </ol>
     */
    public static GateSet recommended() {
        return GateSet.of(
                uniqueFrames(),
                timeSynced(0.5),
                maxAge(0.5),
                notFromFuture(0.05),
                excludedTags(),
                insideField(0.5),
                maxHeight(0.3),
                maxTilt(20),
                maxSingleTagAmbiguity(0.3),
                maxDistance(4.5, 7.0),
                maxReprojection(3.0),
                maxYawRate(3.5),
                headingAgreesWithGyro(15),
                innovation(new InnovationConfig()));
    }

    /** Each camera's frame (sequence ID) at most once. Stateful. */
    public static Gate uniqueFrames() {
        return new Gate() {
            private final Map<String, Long> last = new HashMap<>();
            private final Map<String, Long> loopFirst = new HashMap<>();

            @Override
            public String name() {
                return "uniqueFrames";
            }

            @Override
            public void beginLoop(GateContext ctx) {
                // Candidates from one frame share its sequence ID: compare with the previous loops'.
                loopFirst.clear();
            }

            @Override
            public String reject(PoseObservation obs, GateContext ctx) {
                String cam = obs.camera();
                long seq = obs.frame().sequenceId();
                Long seenThisLoop = loopFirst.get(cam);
                if (seenThisLoop != null && seq <= seenThisLoop) return null; // same frame, another candidate
                Long prev = last.get(cam);
                // PhotonVision restarting resets sequence IDs to 0: a big drop starts over.
                if (prev != null && seq <= prev && prev - seq < 1000) return "frame " + seq + " already used";
                last.put(cam, seq);
                loopFirst.merge(cam, seq, Math::max);
                return null;
            }
        };
    }

    /**
     * PhotonVision's clock was synced to the robot's within {@code maxSeconds} when it sent the frame
     * ({@code timeSinceLastPong}). Right after a reconnect timestamps can be off by seconds.
     */
    public static Gate timeSynced(double maxSeconds) {
        long maxMicros = (long) (maxSeconds * 1e6);
        return Gate.of(
                "timeSynced",
                o -> o.frame().timeSinceLastPongMicros() < 0 || o.frame().timeSinceLastPongMicros() <= maxMicros,
                "time sync older than " + maxSeconds + " s");
    }

    /** Captured at most {@code maxSeconds} ago. */
    public static Gate maxAge(double maxSeconds) {
        return new Gate() {
            @Override
            public String name() {
                return "maxAge";
            }

            @Override
            public String reject(PoseObservation o, GateContext ctx) {
                double age = ctx.nowSeconds() - o.timestampSeconds();
                return age <= maxSeconds ? null : String.format("%.3f s old", age);
            }
        };
    }

    /** Not captured more than {@code toleranceSeconds} in the future (a broken time sync). */
    public static Gate notFromFuture(double toleranceSeconds) {
        return new Gate() {
            @Override
            public String name() {
                return "notFromFuture";
            }

            @Override
            public String reject(PoseObservation o, GateContext ctx) {
                double ahead = o.timestampSeconds() - ctx.nowSeconds();
                return ahead <= toleranceSeconds ? null : String.format("%.3f s in the future", ahead);
            }
        };
    }

    /** No tag in {@link GateContext#excludedTags()} (yours plus the Jetson's list). */
    public static Gate excludedTags() {
        return new Gate() {
            @Override
            public String name() {
                return "excludedTags";
            }

            @Override
            public String reject(PoseObservation o, GateContext ctx) {
                for (int id : o.tagIds()) if (ctx.excludedTags().contains(id)) return "tag " + id + " is excluded";
                return null;
            }
        };
    }

    /** Only these tag IDs (e.g. your alliance's, or the ones on the field elements you trust). */
    public static Gate onlyTags(Supplier<Set<Integer>> allowed) {
        return new Gate() {
            @Override
            public String name() {
                return "onlyTags";
            }

            @Override
            public String reject(PoseObservation o, GateContext ctx) {
                var ok = allowed.get();
                for (int id : o.tagIds()) if (!ok.contains(id)) return "tag " + id + " not allowed";
                return null;
            }
        };
    }

    /** The robot inside the field (the layout's length and width), with {@code marginMeters}. */
    public static Gate insideField(double marginMeters) {
        return new Gate() {
            @Override
            public String name() {
                return "insideField";
            }

            @Override
            public String reject(PoseObservation o, GateContext ctx) {
                double x = o.robotPose().getX(), y = o.robotPose().getY();
                double l = ctx.field().getFieldLength(), w = ctx.field().getFieldWidth();
                boolean in = x >= -marginMeters && x <= l + marginMeters && y >= -marginMeters && y <= w + marginMeters;
                return in ? null : String.format("(%.2f, %.2f) outside the field", x, y);
            }
        };
    }

    /** |robot z| (pose origin height) at most {@code meters}. Only full-pose solves have a height. */
    public static Gate maxHeight(double meters) {
        return Gate.of(
                "maxHeight",
                o -> o.type() == Type.TAG_TRIG || Math.abs(o.robotPose().getZ()) <= meters,
                "robot off the floor by more than " + meters + " m");
    }

    /** Roll and pitch each at most {@code degrees}. Only full-pose solves have them. */
    public static Gate maxTilt(double degrees) {
        double r = Math.toRadians(degrees);
        return Gate.of(
                "maxTilt",
                o -> o.type() == Type.TAG_TRIG
                        || (Math.abs(o.robotPose().getRotation().getX()) <= r && Math.abs(o.robotPose().getRotation().getY()) <= r),
                "robot tilted more than " + degrees + "°");
    }

    /** One tag's full-pose solves: ambiguity at most {@code max} (gyro-checked solves exempt). */
    public static Gate maxSingleTagAmbiguity(double max) {
        return Gate.of(
                "maxAmbiguity",
                o -> o.type() != Type.SINGLE_TAG || (o.ambiguity() >= 0 && o.ambiguity() <= max),
                "single-tag ambiguity over " + max);
    }

    /** Mean tag distance: at most {@code singleTagMeters} for one tag, {@code multiTagMeters} for more. */
    public static Gate maxDistance(double singleTagMeters, double multiTagMeters) {
        return new Gate() {
            @Override
            public String name() {
                return "maxDistance";
            }

            @Override
            public String reject(PoseObservation o, GateContext ctx) {
                double max = o.tagCount() > 1 ? multiTagMeters : singleTagMeters;
                double d = o.averageDistanceMeters();
                return Double.isNaN(d) || d <= max ? null : String.format("%.2f m > %.2f m", d, max);
            }
        };
    }

    /** Reprojection error at most {@code px} (passes when unknown). */
    public static Gate maxReprojection(double px) {
        return Gate.of(
                "maxReprojection",
                o -> Double.isNaN(o.reprojErrPx()) || o.reprojErrPx() <= px,
                "reprojection error over " + px + " px");
    }

    /** Every tag at least {@code px} from the image edge (passes when unknown). Prefer the trust falloff. */
    public static Gate minEdgeDistance(double px) {
        return Gate.of(
                "minEdgeDistance",
                o -> Double.isNaN(o.minEdgePx()) || o.minEdgePx() >= px,
                "a tag within " + px + " px of the image edge");
    }

    /** Every tag's decision margin at least {@code margin} (SpectrumJetson only; passes when unknown). */
    public static Gate minDecisionMargin(double margin) {
        return Gate.of(
                "minDecisionMargin",
                o -> Double.isNaN(o.minDecisionMargin()) || o.minDecisionMargin() >= margin,
                "decision margin under " + margin);
    }

    /** Gyro turn rate at capture at most {@code radPerSec} (passes without a gyro sample). */
    public static Gate maxYawRate(double radPerSec) {
        return new Gate() {
            @Override
            public String name() {
                return "maxYawRate";
            }

            @Override
            public String reject(PoseObservation o, GateContext ctx) {
                var w = ctx.motion().yawRateAt(o.timestampSeconds());
                return w.isEmpty() || Math.abs(w.get()) <= radPerSec
                        ? null
                        : String.format("turning %.1f rad/s", w.get());
            }
        };
    }

    /** Robot speed at capture at most {@code metersPerSec} (passes without a sample). */
    public static Gate maxSpeed(double metersPerSec) {
        return new Gate() {
            @Override
            public String name() {
                return "maxSpeed";
            }

            @Override
            public String reject(PoseObservation o, GateContext ctx) {
                var v = ctx.motion().speedAt(o.timestampSeconds());
                return v.isEmpty() || Math.abs(v.get()) <= metersPerSec ? null : String.format("moving %.1f m/s", v.get());
            }
        };
    }

    /**
     * A full-pose solve's heading within {@code degrees} of the gyro at capture (passes without a
     * gyro sample; the trig solve uses the gyro heading, so it always agrees).
     */
    public static Gate headingAgreesWithGyro(double degrees) {
        double r = Math.toRadians(degrees);
        return new Gate() {
            @Override
            public String name() {
                return "headingAgreesWithGyro";
            }

            @Override
            public String reject(PoseObservation o, GateContext ctx) {
                if (!o.headingFromVision()) return null;
                // From the gyro history in the context (the candidate's own value needs the solver
                // to have had the history).
                var gyro = ctx.motion().headingAt(o.timestampSeconds());
                double e = gyro.isPresent()
                        ? Math.abs(MathUtil.angleModulus(o.robotPose().getRotation().getZ() - gyro.get().getRadians()))
                        : o.headingDisagreementRad();
                return Double.isNaN(e) || e <= r ? null : String.format("heading %.0f° from the gyro", Math.toDegrees(e));
            }
        };
    }

    /** Settings for {@link #innovation}. */
    public static final class InnovationConfig {
        /** Furthest a candidate may be from the current estimate. */
        public double maxTranslationMeters = 1.0;
        /** Largest heading difference (full-pose candidates only). */
        public double maxRotationRad = Math.toRadians(30);
        /** Accept anyway after this many rejected candidates in a row that agree with each other... */
        public int agreeingToReset = 3;
        /** ...within this distance of each other... */
        public double agreeWithinMeters = 0.3;
        /** ...within this long. */
        public double agreeWindowSeconds = 1.0;
        /** While disabled, accept every candidate (the robot was just placed or pushed). */
        public boolean acceptAllWhileDisabled = true;

        public InnovationConfig maxTranslationMeters(double v) {
            maxTranslationMeters = v;
            return this;
        }

        public InnovationConfig maxRotationDeg(double v) {
            maxRotationRad = Math.toRadians(v);
            return this;
        }

        public InnovationConfig agreeingToReset(int n) {
            agreeingToReset = n;
            return this;
        }

        public InnovationConfig acceptAllWhileDisabled(boolean v) {
            acceptAllWhileDisabled = v;
            return this;
        }
    }

    /**
     * No sudden jump away from the pose estimator's current estimate. A bump, wheel slip or a
     * collision can move the robot further than this between frames, so several candidates in a row
     * that agree with each other win: the estimate is wrong, not the cameras. Needs the estimate
     * ({@code VisionSystem.Builder.currentEstimate}); passes without it. Stateful.
     */
    public static Gate innovation(InnovationConfig cfg) {
        return new Gate() {
            private final ArrayDeque<PoseObservation> rejected = new ArrayDeque<>();

            @Override
            public String name() {
                return "innovation";
            }

            @Override
            public String reject(PoseObservation o, GateContext ctx) {
                Pose2d est = ctx.currentEstimate();
                if (est == null || (cfg.acceptAllWhileDisabled && !ctx.enabled())) return null;
                Pose2d p = o.robotPose().toPose2d();
                double dt = p.getTranslation().getDistance(est.getTranslation());
                double dr = Math.abs(MathUtil.angleModulus(p.getRotation().minus(est.getRotation()).getRadians()));
                boolean close = dt <= cfg.maxTranslationMeters && (!o.headingFromVision() || dr <= cfg.maxRotationRad);
                if (close) return null;
                // Rejected: do the last few rejections agree with this one?
                while (!rejected.isEmpty() && o.timestampSeconds() - rejected.peekFirst().timestampSeconds() > cfg.agreeWindowSeconds) {
                    rejected.pollFirst();
                }
                rejected.addLast(o);
                if (rejected.size() >= cfg.agreeingToReset && agree(cfg)) {
                    rejected.clear();
                    return null; // the estimate has drifted: let vision pull it back
                }
                return String.format("%.2f m / %.0f° from the estimate", dt, Math.toDegrees(dr));
            }

            private boolean agree(InnovationConfig cfg) {
                var last = rejected.peekLast().robotPose().toPose2d().getTranslation();
                int n = 0;
                for (var it = rejected.descendingIterator(); it.hasNext() && n < cfg.agreeingToReset; n++) {
                    if (it.next().robotPose().toPose2d().getTranslation().getDistance(last) > cfg.agreeWithinMeters) return false;
                }
                return true;
            }

            @Override
            public void accepted(PoseObservation o, GateContext ctx) {
                rejected.clear();
            }
        };
    }
}
