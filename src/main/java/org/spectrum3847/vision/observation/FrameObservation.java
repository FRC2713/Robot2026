package org.spectrum3847.vision.observation;

import edu.wpi.first.math.geometry.Transform3d;
import java.util.List;

/**
 * One camera frame with tags in it, as the camera reported it: the tags, and PhotonVision's
 * multi-tag solve if it ran. This is the library's input; everything after it (solving, gating,
 * trust) is deterministic, so logged frames replay exactly.
 *
 * @param camera the camera's name (as in PhotonVision)
 * @param sequenceId PhotonVision's frame sequence ID (to drop duplicates, and to match the Jetson's
 *     per-tag quality)
 * @param timestampSeconds capture time (mid-exposure on SpectrumJetson), in the robot's timebase:
 *     PhotonLib's {@code getTimestampSeconds()}, the same clock as {@code Timer.getTimestamp()}
 * @param timeSinceLastPongMicros PhotonVision's time-sync age when it sent the frame; large means
 *     the timestamp may not be synced (after a reconnect)
 * @param multiTagFieldToCamera PhotonVision's multi-tag solve (field to camera), or null
 * @param multiTagReprojErrPx its RMS reprojection error, pixels (NaN without multi-tag)
 * @param multiTagIds the tags the multi-tag solve used (empty without multi-tag)
 * @param tags every tag reported in the frame, in PhotonVision's order
 */
public record FrameObservation(
        String camera,
        long sequenceId,
        double timestampSeconds,
        long timeSinceLastPongMicros,
        Transform3d multiTagFieldToCamera,
        double multiTagReprojErrPx,
        List<Integer> multiTagIds,
        List<TagObservation> tags) {

    public FrameObservation {
        multiTagIds = List.copyOf(multiTagIds);
        tags = List.copyOf(tags);
    }

    public boolean hasMultiTag() {
        return multiTagFieldToCamera != null;
    }

    /** The tag with this ID, or null. */
    public TagObservation tag(int id) {
        for (var t : tags) if (t.id() == id) return t;
        return null;
    }
}
