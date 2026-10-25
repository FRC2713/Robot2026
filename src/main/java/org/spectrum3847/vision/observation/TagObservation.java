package org.spectrum3847.vision.observation;

import edu.wpi.first.math.geometry.Transform3d;

/**
 * One AprilTag seen in one camera frame: everything the solvers, gates and trust models need,
 * and nothing else. Small and plain, so it can be logged and replayed (see {@link
 * ObservationCodec}).
 *
 * <p>Quality values are NaN when unknown. SpectrumJetson's PhotonVision measures them on the Jetson
 * ({@code /photonvision/<camera>/tagQuality}); with stock PhotonVision, {@link
 * org.spectrum3847.vision.io.TagQualityEstimator} computes all of them except the decision margin
 * from the corners and the camera calibration PhotonLib provides.
 *
 * @param id fiducial ID
 * @param bestCameraToTag the lower-error single-tag solution (PhotonLib convention: camera frame
 *     x forward, y left, z up; tag frame x out of the tag's face)
 * @param altCameraToTag the other solution (equal to best for tags solved by multi-tag)
 * @param ambiguity PhotonVision's pose ambiguity: ratio of the two solutions' errors, 0..1; -1 if
 *     unknown
 * @param yawDeg tag centre yaw in the image, degrees (positive left in PhotonLib 2026)
 * @param pitchDeg tag centre pitch, degrees
 * @param area tag area, percent of the image
 * @param corners detected corners, x0, y0, ... x3, y3, in raw (distorted) pixels, detection order
 * @param decisionMargin the detector's decision margin (Jetson only)
 * @param edgePx distance of the nearest corner from the image edge, pixels
 * @param undistortPx how far the lens model moved the corner that moved most, pixels
 * @param reprojBestPx RMS reprojection error of the best solution, pixels
 * @param reprojAltPx RMS reprojection error of the alternate solution, pixels (NaN: no alternate)
 */
public record TagObservation(
        int id,
        Transform3d bestCameraToTag,
        Transform3d altCameraToTag,
        double ambiguity,
        double yawDeg,
        double pitchDeg,
        double area,
        double[] corners,
        double decisionMargin,
        double edgePx,
        double undistortPx,
        double reprojBestPx,
        double reprojAltPx) {

    /** Straight-line distance from the camera to the tag, metres (best solution). */
    public double distanceMeters() {
        return bestCameraToTag.getTranslation().getNorm();
    }

    /** A copy with the quality values replaced (NaN keeps the current value). */
    public TagObservation withQuality(
            double decisionMargin, double edgePx, double undistortPx, double reprojBestPx, double reprojAltPx) {
        return new TagObservation(
                id,
                bestCameraToTag,
                altCameraToTag,
                ambiguity,
                yawDeg,
                pitchDeg,
                area,
                corners,
                Double.isNaN(decisionMargin) ? this.decisionMargin : decisionMargin,
                Double.isNaN(edgePx) ? this.edgePx : edgePx,
                Double.isNaN(undistortPx) ? this.undistortPx : undistortPx,
                Double.isNaN(reprojBestPx) ? this.reprojBestPx : reprojBestPx,
                Double.isNaN(reprojAltPx) ? this.reprojAltPx : reprojAltPx);
    }
}
