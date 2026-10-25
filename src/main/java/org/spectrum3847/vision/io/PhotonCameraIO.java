package org.spectrum3847.vision.io;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import org.spectrum3847.vision.compat.CameraCompat;
import org.spectrum3847.vision.compat.NtOptions;
import org.spectrum3847.vision.observation.FrameObservation;
import org.spectrum3847.vision.observation.TagObservation;

/**
 * A real (or PhotonLib-simulated) camera: PhotonLib's results, plus SpectrumJetson's per-tag
 * quality when the coprocessor publishes it, plus robot-side quality from the camera's calibration
 * otherwise.
 *
 * <p>Works with stock PhotonVision too. It also asks PhotonVision to keep the camera enabled at
 * start: a camera that robot code disabled stays disabled across a robot-code restart.
 */
public class PhotonCameraIO implements CameraIO {
    private final PhotonCamera camera;
    private final DoubleArraySubscriber quality;
    private final TagQualityMatcher matcher = new TagQualityMatcher();
    private TagQualityEstimator estimator = null;
    private double lastCalibrationCheck = -1e9;
    private final boolean robotSideQuality;

    /** @param name the camera's name exactly as PhotonVision shows it */
    public static PhotonCameraIO of(String name) {
        return new PhotonCameraIO(new PhotonCamera(name), true);
    }

    /**
     * @param camera the camera
     * @param robotSideQuality compute missing per-tag quality on the robot (recommended; ~10 µs a tag)
     */
    public PhotonCameraIO(PhotonCamera camera, boolean robotSideQuality) {
        this.camera = camera;
        this.robotSideQuality = robotSideQuality;
        quality =
                NetworkTableInstance.getDefault()
                        .getTable("photonvision")
                        .getSubTable(camera.getName())
                        .getDoubleArrayTopic("tagQuality")
                        .subscribe(new double[0], NtOptions.everyValue(32));
        CameraCompat.requestEnabled(camera, true);
    }

    public PhotonCamera camera() {
        return camera;
    }

    @Override
    public String name() {
        return camera.getName();
    }

    @Override
    public void update(Inputs inputs) {
        inputs.connected = camera.isConnected();
        for (var v : quality.readQueue()) matcher.offer(v.value);
        if (robotSideQuality) refreshEstimator();
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        inputs.resultCount = results.size();
        inputs.frames = new ArrayList<>();
        for (var r : results) {
            if (!r.hasTargets()) continue;
            FrameObservation f = matcher.apply(toFrame(camera.getName(), r));
            if (estimator != null) f = estimator.fill(f);
            inputs.frames.add(f);
        }
        inputs.qualityMatched = matcher.matchedCount();
    }

    // The calibration can arrive after start, or change: look again every 5 s.
    private void refreshEstimator() {
        double now = org.spectrum3847.vision.compat.Clock.now();
        if (now - lastCalibrationCheck < 5) return;
        lastCalibrationCheck = now;
        var k = camera.getCameraMatrix();
        var d = camera.getDistCoeffs();
        if (k.isPresent() && d.isPresent()) {
            estimator = new TagQualityEstimator(k.get().getData(), d.get().getData());
        }
    }

    /** One PhotonLib result as a frame (no quality values yet). */
    public static FrameObservation toFrame(String camera, PhotonPipelineResult r) {
        var tags = new ArrayList<TagObservation>();
        for (PhotonTrackedTarget t : r.getTargets()) {
            if (t.getFiducialId() < 0) continue;
            double[] corners = new double[8];
            var cs = t.getDetectedCorners();
            if (cs != null && cs.size() == 4) {
                for (int i = 0; i < 4; i++) {
                    TargetCorner c = cs.get(i);
                    corners[2 * i] = c.x;
                    corners[2 * i + 1] = c.y;
                }
            } else {
                java.util.Arrays.fill(corners, Double.NaN);
            }
            tags.add(new TagObservation(
                    t.getFiducialId(),
                    t.getBestCameraToTarget(),
                    t.getAlternateCameraToTarget(),
                    t.getPoseAmbiguity(),
                    t.getYaw(),
                    t.getPitch(),
                    t.getArea(),
                    corners,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN));
        }
        Transform3d multi = null;
        double multiErr = Double.NaN;
        var ids = new ArrayList<Integer>();
        var mt = r.getMultiTagResult();
        if (mt.isPresent()) {
            multi = mt.get().estimatedPose.best;
            multiErr = mt.get().estimatedPose.bestReprojErr;
            for (Short id : mt.get().fiducialIDsUsed) ids.add((int) id);
        }
        return new FrameObservation(
                camera,
                r.metadata.getSequenceID(),
                r.getTimestampSeconds(),
                r.metadata.timeSinceLastPong,
                multi,
                multiErr,
                ids,
                tags);
    }
}
