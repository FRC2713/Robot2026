package org.spectrum3847.vision.trust;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.spectrum3847.vision.gate.GateContext;
import org.spectrum3847.vision.observation.PoseObservation;
import org.spectrum3847.vision.observation.PoseObservation.Type;

/**
 * How much the pose estimator should trust one accepted pose: standard deviations for x, y
 * (metres) and heading (radians), as WPILib's {@code addVisionMeasurement} and CTRE's take them.
 * Smaller means trusted more.
 *
 * <p>{@link #recommended()} grows with distance squared over the number of tags, the shape most
 * strong teams use (the error of a tag's position estimate grows roughly with distance squared),
 * then multiplies in everything else that makes a measurement worse: tags near the image edge, a
 * high reprojection error, a low decision margin, single-tag ambiguity, and the robot moving or
 * turning. Every factor is a field you can change; README "Trust" explains each.
 */
public class TrustModel {
    /** x/y standard deviation for one tag at 1 m, metres. */
    public double xyPerMeterSquared = 0.02;
    /** Heading standard deviation for multi-tag at 1 m, radians. */
    public double thetaPerMeterSquared = 0.06;
    /** Floors, so a close tag never claims centimetre accuracy. */
    public double minXy = 0.02;
    public double minTheta = Math.toRadians(1.5);
    /** Heading standard deviation for solves whose heading isn't worth using: effectively ignored. */
    public double ignoredTheta = 1e6;
    /** Single-tag full-pose solves: heading ignored (it's the least reliable part of one tag). */
    public boolean ignoreSingleTagHeading = true;
    /** The gyro trig solve's x/y relative to a full-pose single-tag solve (it's better at range). */
    public double trigXyFactor = 0.8;
    /** Edge falloff: below this many pixels from the edge, trust drops... */
    public double edgeSoftPx = 60;
    /** ...up to this factor at the very edge. */
    public double edgeMaxFactor = 3.0;
    /** Reprojection: factor 1 + error / this (pixels). */
    public double reprojScalePx = 1.5;
    /** Decision margin (SpectrumJetson): below this, factor softMargin / margin. */
    public double softDecisionMargin = 40;
    /** Ambiguity (single tag): factor 1 + this * ambiguity. */
    public double ambiguityGain = 4.0;
    /** Motion: factor 1 + speed * this (per m/s)... */
    public double perMeterPerSecond = 0.15;
    /** ...+ |turn rate| * this (per rad/s). */
    public double perRadianPerSecond = 0.4;

    /** Our defaults (the fields above). */
    public static TrustModel recommended() {
        return new TrustModel();
    }

    /**
     * The standard deviations for {@code o}, times {@code cameraFactor} (a camera's own multiplier:
     * a poorly calibrated or wobbly camera trusted less).
     */
    public Matrix<N3, N1> stdDevs(PoseObservation o, GateContext ctx, double cameraFactor) {
        int n = Math.max(1, o.tagCount());
        double d = Double.isNaN(o.averageDistanceMeters()) ? 3.0 : o.averageDistanceMeters();
        double f = cameraFactor * qualityFactor(o) * motionFactor(o, ctx);
        double xy = Math.max(minXy, xyPerMeterSquared * d * d / n) * f;
        double theta = Math.max(minTheta, thetaPerMeterSquared * d * d / n) * f;
        if (o.type() == Type.TAG_TRIG) {
            xy *= trigXyFactor;
            theta = ignoredTheta; // the heading is the gyro's own
        } else if (o.tagCount() <= 1 && ignoreSingleTagHeading) {
            theta = ignoredTheta;
        }
        return VecBuilder.fill(xy, xy, theta);
    }

    /** Edge, reprojection, decision margin and ambiguity together (1 = perfect). */
    public double qualityFactor(PoseObservation o) {
        double f = 1;
        if (!Double.isNaN(o.minEdgePx()) && o.minEdgePx() < edgeSoftPx) {
            f *= 1 + (edgeMaxFactor - 1) * (edgeSoftPx - Math.max(0, o.minEdgePx())) / edgeSoftPx;
        }
        if (!Double.isNaN(o.reprojErrPx())) f *= 1 + o.reprojErrPx() / reprojScalePx;
        if (!Double.isNaN(o.minDecisionMargin()) && o.minDecisionMargin() < softDecisionMargin) {
            f *= softDecisionMargin / Math.max(5, o.minDecisionMargin());
        }
        if (o.tagCount() <= 1 && o.ambiguity() > 0) f *= 1 + ambiguityGain * o.ambiguity();
        return f;
    }

    /** Robot speed and turn rate at capture (1 when no gyro history). */
    public double motionFactor(PoseObservation o, GateContext ctx) {
        double v = ctx.motion().speedAt(o.timestampSeconds()).orElse(0.0);
        double w = ctx.motion().yawRateAt(o.timestampSeconds()).orElse(0.0);
        return 1 + Math.abs(v) * perMeterPerSecond + Math.abs(w) * perRadianPerSecond;
    }
}
