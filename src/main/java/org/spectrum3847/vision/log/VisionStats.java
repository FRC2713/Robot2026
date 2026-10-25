package org.spectrum3847.vision.log;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.TreeMap;
import org.spectrum3847.vision.VisionUpdate;

/**
 * Counts since start (or {@link #reset()}): frames, candidates, accepted, and rejections by gate,
 * per camera and in total. The quickest way to tune gates: run a match (or a replay), read {@link
 * #summary()}, change one limit, run it again.
 */
public class VisionStats {
    /** Per camera (and "all"). */
    public static final class Counts {
        public long frames, candidates, accepted;
        public final Map<String, Long> rejectedBy = new TreeMap<>();
        public double lastAcceptedSeconds = Double.NEGATIVE_INFINITY;

        public long rejected() {
            return rejectedBy.values().stream().mapToLong(Long::longValue).sum();
        }
    }

    private final Map<String, Counts> byCamera = new LinkedHashMap<>();
    private final Counts all = new Counts();

    public synchronized void record(VisionUpdate u) {
        for (var f : u.frames()) {
            all.frames++;
            camera(f.camera()).frames++;
        }
        for (var a : u.accepted()) {
            for (var c : new Counts[] {all, camera(a.observation().camera())}) {
                c.candidates++;
                c.accepted++;
                c.lastAcceptedSeconds = Math.max(c.lastAcceptedSeconds, a.observation().timestampSeconds());
            }
        }
        for (var r : u.rejected()) {
            for (var c : new Counts[] {all, camera(r.observation().camera())}) {
                c.candidates++;
                c.rejectedBy.merge(r.gate(), 1L, Long::sum);
            }
        }
    }

    public synchronized Counts camera(String name) {
        return byCamera.computeIfAbsent(name, n -> new Counts());
    }

    public synchronized Counts all() {
        return all;
    }

    public synchronized Map<String, Counts> cameras() {
        return new LinkedHashMap<>(byCamera);
    }

    public synchronized void reset() {
        byCamera.clear();
        all.frames = all.candidates = all.accepted = 0;
        all.rejectedBy.clear();
        all.lastAcceptedSeconds = Double.NEGATIVE_INFINITY;
    }

    /** One line per camera: "TopLeft: 812 frames, 790 candidates, 701 accepted; rejected maxDistance 60, maxYawRate 29". */
    public synchronized String summary() {
        var sb = new StringBuilder();
        line(sb, "all", all);
        byCamera.forEach((n, c) -> line(sb, n, c));
        return sb.toString();
    }

    private static void line(StringBuilder sb, String name, Counts c) {
        sb.append(String.format("%s: %d frames, %d candidates, %d accepted", name, c.frames, c.candidates, c.accepted));
        if (!c.rejectedBy.isEmpty()) {
            sb.append("; rejected");
            c.rejectedBy.forEach((g, n) -> sb.append(' ').append(g).append(' ').append(n).append(','));
            sb.setLength(sb.length() - 1);
        }
        sb.append('\n');
    }
}
