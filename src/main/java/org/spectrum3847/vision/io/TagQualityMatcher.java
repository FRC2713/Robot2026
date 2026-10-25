package org.spectrum3847.vision.io;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Map;
import org.spectrum3847.vision.observation.FrameObservation;
import org.spectrum3847.vision.observation.TagObservation;

/**
 * Pairs SpectrumJetson's per-tag quality ({@code /photonvision/<camera>/tagQuality}) with
 * PhotonLib's results by frame sequence ID.
 *
 * <p>The topic's value: {@code [sequenceID, tagCount, then per tag: fiducialId, decisionMargin,
 * edgePx, undistortPx, reprojBestPx, reprojAltPx]}, sent just before the result it belongs to, only
 * for frames with tags. Stock PhotonVision never publishes it: then nothing matches and the
 * quality values stay as they were.
 */
public class TagQualityMatcher {
    public static final int FIELDS_PER_TAG = 6;
    private final int capacity;
    private final Map<Long, double[]> pending =
            new LinkedHashMap<>(64, 0.75f, false) {
                @Override
                protected boolean removeEldestEntry(Map.Entry<Long, double[]> e) {
                    return size() > capacity;
                }
            };
    private long matched = 0, unmatched = 0;

    /** @param capacity quality arrays kept waiting for their result (a few frames is plenty) */
    public TagQualityMatcher(int capacity) {
        this.capacity = capacity;
    }

    public TagQualityMatcher() {
        this(32);
    }

    /** A {@code tagQuality} value from NetworkTables. Malformed values are ignored. */
    public synchronized void offer(double[] value) {
        if (value == null || value.length < 2) return;
        int n = (int) value[1];
        if (n < 0 || value.length != 2 + FIELDS_PER_TAG * n) return;
        pending.put((long) value[0], value);
    }

    /**
     * {@code frame} with each tag's quality filled in from the matching array (by tag ID), or
     * unchanged if there's none.
     */
    public synchronized FrameObservation apply(FrameObservation frame) {
        double[] q = pending.remove(frame.sequenceId());
        if (q == null) {
            if (!frame.tags().isEmpty()) unmatched++;
            return frame;
        }
        matched++;
        int n = (int) q[1];
        var tags = new ArrayList<TagObservation>(frame.tags().size());
        for (var t : frame.tags()) {
            TagObservation filled = t;
            for (int i = 0; i < n; i++) {
                int base = 2 + FIELDS_PER_TAG * i;
                if ((int) q[base] == t.id()) {
                    filled = t.withQuality(q[base + 1], q[base + 2], q[base + 3], q[base + 4], q[base + 5]);
                    break;
                }
            }
            tags.add(filled);
        }
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

    /** Frames with tags that got (didn't get) quality values: unmatched > 0 with matched = 0 means stock PhotonVision. */
    public synchronized long matchedCount() {
        return matched;
    }

    public synchronized long unmatchedCount() {
        return unmatched;
    }
}
