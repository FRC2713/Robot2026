package org.spectrum3847.vision.io;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import org.spectrum3847.vision.observation.FrameObservation;
import org.spectrum3847.vision.observation.TagObservation;

/**
 * Per-tag quality on the robot, for PhotonVision without SpectrumJetson's {@code tagQuality} topic:
 * from the corners and the camera calibration PhotonLib already gives ({@code
 * PhotonCamera.getCameraMatrix()} and {@code getDistCoeffs()}). Computes everything the Jetson does
 * except the decision margin. Only fills values that are NaN, so the Jetson's own win when present.
 *
 * <ul>
 *   <li>edgePx: the corner nearest the image edge. The image size isn't published; it's taken as
 *       twice the principal point (cx, cy), which is within a few pixels for a calibrated camera.
 *   <li>undistortPx: how far undistortion moved the corner that moved most (OpenCV's 8-coefficient
 *       model, the one PhotonVision calibrates, inverted iteratively).
 *   <li>reprojBestPx / reprojAltPx: RMS pixel error of the tag's corners under each solution.
 * </ul>
 */
public class TagQualityEstimator {
    private final double fx, fy, cx, cy;
    private final double k1, k2, p1, p2, k3, k4, k5, k6;
    private final double tagSize;

    /**
     * @param cameraMatrix fx, 0, cx, 0, fy, cy, 0, 0, 1 (row-major, PhotonLib's order)
     * @param distCoeffs k1, k2, p1, p2, k3, k4, k5, k6 (fewer is fine: the rest are 0)
     * @param tagSizeMeters the tags' black-square size (FRC 2024 on: 6.5 in)
     */
    public TagQualityEstimator(double[] cameraMatrix, double[] distCoeffs, double tagSizeMeters) {
        fx = cameraMatrix[0];
        cx = cameraMatrix[2];
        fy = cameraMatrix[4];
        cy = cameraMatrix[5];
        double[] d = new double[8];
        for (int i = 0; i < Math.min(8, distCoeffs.length); i++) d[i] = distCoeffs[i];
        k1 = d[0];
        k2 = d[1];
        p1 = d[2];
        p2 = d[3];
        k3 = d[4];
        k4 = d[5];
        k5 = d[6];
        k6 = d[7];
        tagSize = tagSizeMeters;
    }

    public TagQualityEstimator(double[] cameraMatrix, double[] distCoeffs) {
        this(cameraMatrix, distCoeffs, Units.inchesToMeters(6.5));
    }

    /** {@code frame} with every tag's missing quality values filled in. */
    public FrameObservation fill(FrameObservation frame) {
        boolean any = false;
        var tags = new ArrayList<TagObservation>(frame.tags().size());
        for (var t : frame.tags()) {
            var f = fill(t);
            any |= f != t;
            tags.add(f);
        }
        if (!any) return frame;
        return new FrameObservation(
                frame.camera(),
                frame.sequenceId(),
                frame.timestampSeconds(),
                frame.timeSinceLastPongMicros(),
                frame.multiTagFieldToCamera(),
                frame.multiTagReprojErrPx(),
                frame.multiTagIds(),
                tags);
    }

    public TagObservation fill(TagObservation t) {
        double[] c = t.corners();
        if (c == null || c.length != 8 || Double.isNaN(c[0])) return t;
        if (!Double.isNaN(t.edgePx()) && !Double.isNaN(t.undistortPx()) && !Double.isNaN(t.reprojBestPx())) return t;
        double w = 2 * cx, h = 2 * cy;
        double edge = Double.POSITIVE_INFINITY, moved = 0;
        double[] u = new double[8];
        for (int i = 0; i < 4; i++) {
            double x = c[2 * i], y = c[2 * i + 1];
            edge = Math.min(edge, Math.min(Math.min(x, y), Math.min(w - 1 - x, h - 1 - y)));
            double[] p = undistort(x, y);
            u[2 * i] = p[0];
            u[2 * i + 1] = p[1];
            moved = Math.max(moved, Math.hypot(p[0] - x, p[1] - y));
        }
        double best = reprojection(t.bestCameraToTag(), u);
        double alt = t.altCameraToTag().equals(t.bestCameraToTag()) ? Double.NaN : reprojection(t.altCameraToTag(), u);
        return t.withQuality(Double.NaN, edge, moved, best, alt);
    }

    /** OpenCV's undistortPoints for one point, in pixels (P = the camera matrix). */
    public double[] undistort(double u, double v) {
        double x0 = (u - cx) / fx, y0 = (v - cy) / fy, x = x0, y = y0;
        for (int i = 0; i < 20; i++) {
            double r2 = x * x + y * y;
            double icdist = (1 + ((k6 * r2 + k5) * r2 + k4) * r2) / (1 + ((k3 * r2 + k2) * r2 + k1) * r2);
            double dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
            double dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
            x = (x0 - dx) * icdist;
            y = (y0 - dy) * icdist;
        }
        return new double[] {x * fx + cx, y * fy + cy};
    }

    /** The OpenCV model forward: undistorted pixel to the raw pixel the camera records. */
    public double[] distort(double u, double v) {
        double x = (u - cx) / fx, y = (v - cy) / fy, r2 = x * x + y * y;
        double radial = (1 + ((k3 * r2 + k2) * r2 + k1) * r2) / (1 + ((k6 * r2 + k5) * r2 + k4) * r2);
        double xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
        double yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
        return new double[] {xd * fx + cx, yd * fy + cy};
    }

    /**
     * Pinhole projection of a camera-frame point (x forward, y left, z up) to an undistorted
     * pixel, or null if it's behind the camera.
     */
    public double[] project(Translation3d p) {
        if (p.getX() <= 1e-6) return null;
        return new double[] {cx - fx * p.getY() / p.getX(), cy - fy * p.getZ() / p.getX()};
    }

    /** The tag's corners in its own frame, in detection order (PhotonLib's TargetModel). */
    public Translation3d[] tagCorners() {
        double s = tagSize / 2;
        return new Translation3d[] {
            new Translation3d(0, -s, -s), new Translation3d(0, s, -s), new Translation3d(0, s, s), new Translation3d(0, -s, s)
        };
    }

    private double reprojection(Transform3d cameraToTag, double[] undistorted) {
        double sum = 0;
        var corners = tagCorners();
        for (int i = 0; i < 4; i++) {
            var p = corners[i].rotateBy(cameraToTag.getRotation()).plus(cameraToTag.getTranslation());
            double[] px = project(p);
            if (px == null) return Double.NaN;
            double du = px[0] - undistorted[2 * i], dv = px[1] - undistorted[2 * i + 1];
            sum += du * du + dv * dv;
        }
        return Math.sqrt(sum / 4);
    }
}
