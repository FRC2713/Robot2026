package org.spectrum3847.vision.testing;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.spectrum3847.vision.io.TagQualityEstimator;
import org.spectrum3847.vision.observation.FrameObservation;
import org.spectrum3847.vision.observation.TagObservation;

/**
 * Frames a camera at a known pose would report, without a camera, PhotonVision or OpenCV: for unit
 * tests of gates, trust and solvers, and for trying a gate change against a scenario ("a robot at
 * (3, 4) spinning at 4 rad/s") before a match. Plain Java.
 *
 * <p>Each visible tag (in front, facing the camera, inside the image, within {@link #maxRange})
 * gets its true camera-to-tag pose (plus {@link #poseNoise}), an alternate solution, corners
 * projected through the lens model (plus {@link #pixelNoise}), and, with two or more tags, a
 * multi-tag field-to-camera pose. Quality values are filled the way the robot-side estimator
 * computes them.
 */
public class SyntheticCamera {
    public final String name;
    public final double fx, fy, cx, cy;
    public final int width, height;
    public final double[] distCoeffs;
    public double maxRange = 8;
    /** Standard deviation of corner noise, pixels. */
    public double pixelNoise = 0;
    /** Standard deviation of camera-to-tag translation noise, metres per metre of distance. */
    public double poseNoise = 0;
    /** The ambiguity each tag reports (PhotonVision's ratio; 0.05 is a clear tag). */
    public double ambiguity = 0.05;
    /** Swap best and alternate solutions (an ambiguity flip: the wrong one looks best). */
    public boolean flipSolutions = false;
    /** Give the multi-tag solve for frames with 2+ tags. */
    public boolean multiTag = true;
    private final Random random;
    private final TagQualityEstimator lens;
    private long sequence = 0;

    /** A 1280x800 camera like a Thriftiest Cam: ~70° horizontal field of view, a mild lens. */
    public static SyntheticCamera typical(String name, long seed) {
        return new SyntheticCamera(
                name, 910, 910, 640, 400, 1280, 800, new double[] {0.05, -0.08, 0.001, -0.001, 0.02, 0, 0, 0}, seed);
    }

    public SyntheticCamera(
            String name, double fx, double fy, double cx, double cy, int width, int height, double[] distCoeffs, long seed) {
        this.name = name;
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;
        this.width = width;
        this.height = height;
        this.distCoeffs = distCoeffs.clone();
        this.random = new Random(seed);
        lens = new TagQualityEstimator(new double[] {fx, 0, cx, 0, fy, cy, 0, 0, 1}, distCoeffs);
    }

    /** The calibration, as PhotonLib gives it (for a TagQualityEstimator). */
    public double[] cameraMatrix() {
        return new double[] {fx, 0, cx, 0, fy, cy, 0, 0, 1};
    }

    /**
     * The frame this camera reports with the robot at {@code robot} (field frame) at {@code
     * timestampSeconds}, or null if it sees no tag.
     */
    public FrameObservation frame(AprilTagFieldLayout field, Pose3d robot, Transform3d robotToCamera, double timestampSeconds) {
        Pose3d camera = robot.plus(robotToCamera);
        var tags = new ArrayList<TagObservation>();
        var ids = new ArrayList<Integer>();
        for (AprilTag tag : field.getTags()) {
            var t = see(camera, tag);
            if (t != null) {
                tags.add(t);
                ids.add(t.id());
            }
        }
        if (tags.isEmpty()) return null;
        Transform3d multi = null;
        double multiErr = Double.NaN;
        List<Integer> used = List.of();
        if (multiTag && tags.size() >= 2) {
            multi = noisy(new Transform3d(new Pose3d(), camera), 0.0);
            multiErr = pixelNoise;
            used = ids;
        }
        return new FrameObservation(name, ++sequence, timestampSeconds, 1000, multi, multiErr, used, tags);
    }

    private TagObservation see(Pose3d camera, AprilTag tag) {
        Transform3d camToTag = new Transform3d(camera, tag.pose);
        Translation3d p = camToTag.getTranslation();
        double d = p.getNorm();
        if (p.getX() < 0.1 || d > maxRange) return null;
        // Facing the camera: the tag's x axis (out of its face) points back toward the camera.
        Translation3d normal = new Translation3d(1, 0, 0).rotateBy(camToTag.getRotation());
        double facing = -(normal.getX() * p.getX() + normal.getY() * p.getY() + normal.getZ() * p.getZ()) / d;
        if (facing < 0.15) return null;
        double[] corners = new double[8];
        var model = lens.tagCorners();
        for (int i = 0; i < 4; i++) {
            var c = model[i].rotateBy(camToTag.getRotation()).plus(p);
            double[] px = lens.project(c);
            if (px == null) return null;
            double[] raw = lens.distort(px[0], px[1]);
            if (raw[0] < 0 || raw[1] < 0 || raw[0] > width - 1 || raw[1] > height - 1) return null;
            corners[2 * i] = raw[0] + pixelNoise * random.nextGaussian();
            corners[2 * i + 1] = raw[1] + pixelNoise * random.nextGaussian();
        }
        Transform3d best = noisy(camToTag, poseNoise * d);
        // The classic planar ambiguity: the same position, the tag tilted the other way.
        double view = Math.acos(Math.min(1, facing));
        Transform3d alt = new Transform3d(best.getTranslation(), best.getRotation().rotateBy(new Rotation3d(0, 0, 2 * view + 0.3)));
        if (flipSolutions) {
            var tmp = best;
            best = alt;
            alt = tmp;
        }
        double yaw = Math.toDegrees(Math.atan2(p.getY(), p.getX()));
        double pitch = Math.toDegrees(Math.atan2(p.getZ(), p.getX()));
        var t = new TagObservation(tag.ID, best, alt, ambiguity, yaw, pitch, Double.NaN, corners,
                Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN);
        return lens.fill(t);
    }

    private Transform3d noisy(Transform3d t, double sigma) {
        if (sigma <= 0) return t;
        return new Transform3d(
                t.getTranslation().plus(new Translation3d(
                        sigma * random.nextGaussian(), sigma * random.nextGaussian(), sigma * random.nextGaussian())),
                t.getRotation());
    }
}
