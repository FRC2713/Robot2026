package org.spectrum3847.vision.testing;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.DoubleFunction;
import org.spectrum3847.vision.VisionSystem;
import org.spectrum3847.vision.VisionUpdate;
import org.spectrum3847.vision.io.FakeCameraIO;
import org.spectrum3847.vision.log.VisionStats;

/**
 * A whole VisionSystem on a pretend robot: synthetic cameras, a robot path, the real solvers, gates
 * and trust. Use it in a unit test to check a gate or trust change does what you meant, or to see
 * what a scenario does ("full speed past the tags", "spinning in place") before trying it on the
 * field.
 *
 * <pre>{@code
 * var s = new VisionScenario(field)
 *     .camera(SyntheticCamera.typical("TopLeft", 1), robotToTopLeft);
 * s.configure(b -> b.gates(Gates.recommended().replace(Gates.maxYawRate(5))));
 * s.run(0, 3, 0.02, t -> new Pose2d(2 + t, 4, Rotation2d.kZero), 0.0, 1.0);
 * System.out.println(s.stats().summary());
 * }</pre>
 */
public class VisionScenario {
    private final AprilTagFieldLayout field;
    private final List<SyntheticCamera> synth = new ArrayList<>();
    private final List<FakeCameraIO> ios = new ArrayList<>();
    private final List<Transform3d> mounts = new ArrayList<>();
    private Consumer<VisionSystem.Builder> configure = b -> {};
    private final List<Measurement> measurements = new ArrayList<>();
    private VisionSystem vision;
    private double now = 0;
    private Pose2d truth = new Pose2d();

    /** One measurement the pose estimator would have received, with the truth at that time. */
    public record Measurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs, Pose2d truth) {
        public double errorMeters() {
            return pose.getTranslation().getDistance(truth.getTranslation());
        }
    }

    public VisionScenario(AprilTagFieldLayout field) {
        this.field = field;
    }

    public VisionScenario camera(SyntheticCamera camera, Transform3d robotToCamera) {
        synth.add(camera);
        ios.add(new FakeCameraIO(camera.name));
        mounts.add(robotToCamera);
        return this;
    }

    /** Change the builder (gates, trust, solver...) before the system is built. */
    public VisionScenario configure(Consumer<VisionSystem.Builder> c) {
        configure = c;
        return this;
    }

    private VisionSystem system() {
        if (vision == null) {
            var b = VisionSystem.builder(field)
                    .offline()
                    .clock(() -> now)
                    .currentEstimate(() -> truth)
                    .sink((pose, t, std) -> measurements.add(new Measurement(pose, t, std, truth)));
            for (int i = 0; i < ios.size(); i++) b.camera(ios.get(i), mounts.get(i), 1.0);
            configure.accept(b);
            vision = b.build();
        }
        return vision;
    }

    /**
     * One robot loop at time {@code t}: the robot at {@code robot}, turning at {@code yawRate} rad/s
     * and moving at {@code speed} m/s; each camera's frame was captured {@code latencySeconds} ago.
     */
    public VisionUpdate step(double t, Pose2d robot, double yawRate, double speed, double latencySeconds) {
        var vs = system();
        now = t;
        truth = robot;
        double captured = t - latencySeconds;
        vs.addMotion(captured, robot.getRotation(), yawRate, speed);
        vs.addMotion(t, robot.getRotation(), yawRate, speed);
        for (int i = 0; i < synth.size(); i++) {
            var f = synth.get(i).frame(field, new Pose3d(robot), mounts.get(i), captured);
            if (f != null) ios.get(i).queue(f);
        }
        return vs.periodic();
    }

    /** Loops from {@code start} to {@code end} every {@code dt} along {@code path(t)}. */
    public VisionScenario run(
            double start, double end, double dt, DoubleFunction<Pose2d> path, double yawRate, double speed) {
        for (double t = start; t <= end + 1e-9; t += dt) step(t, path.apply(t), yawRate, speed, 0.03);
        return this;
    }

    public List<Measurement> measurements() {
        return measurements;
    }

    public VisionStats stats() {
        return system().stats();
    }

    /** Largest error of an accepted measurement against the truth, metres (NaN if none). */
    public double worstErrorMeters() {
        return measurements.stream().mapToDouble(Measurement::errorMeters).max().orElse(Double.NaN);
    }
}
