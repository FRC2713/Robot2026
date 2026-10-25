package org.spectrum3847.vision.observation;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.List;

/**
 * Frames to one flat {@code double[]} and back, for logging camera inputs (AdvantageKit's {@code
 * LogTable.put(String, double[])}, a WPILib DataLog entry, or NetworkTables) and replaying them
 * later through the same solvers, gates and trust model.
 *
 * <p>Layout, version 1: {@code [1, frameCount, frame...]}, each frame {@code [sequenceId, timestamp,
 * timeSinceLastPong, hasMultiTag, multiTag transform (7: x y z qw qx qy qz), multiTagReprojErr,
 * multiTagIdCount, ids..., tagCount, tag...]}, each tag {@code [id, best (7), alt (7), ambiguity,
 * yaw, pitch, area, corners (8), decisionMargin, edgePx, undistortPx, reprojBestPx, reprojAltPx]}
 * (32 numbers). The camera name isn't in it: log each camera under its own key.
 */
public final class ObservationCodec {
    public static final int VERSION = 1;
    private static final int TAG_SIZE = 32; // id, best (7), alt (7), 4 angles/area, corners (8), 5 quality

    private ObservationCodec() {}

    public static double[] encode(List<FrameObservation> frames) {
        var out = new ArrayList<Double>();
        out.add((double) VERSION);
        out.add((double) frames.size());
        for (var f : frames) {
            out.add((double) f.sequenceId());
            out.add(f.timestampSeconds());
            out.add((double) f.timeSinceLastPongMicros());
            out.add(f.hasMultiTag() ? 1.0 : 0.0);
            putTransform(out, f.hasMultiTag() ? f.multiTagFieldToCamera() : new Transform3d());
            out.add(f.multiTagReprojErrPx());
            out.add((double) f.multiTagIds().size());
            for (int id : f.multiTagIds()) out.add((double) id);
            out.add((double) f.tags().size());
            for (var t : f.tags()) {
                out.add((double) t.id());
                putTransform(out, t.bestCameraToTag());
                putTransform(out, t.altCameraToTag());
                out.add(t.ambiguity());
                out.add(t.yawDeg());
                out.add(t.pitchDeg());
                out.add(t.area());
                for (int i = 0; i < 8; i++) out.add(t.corners() != null && t.corners().length == 8 ? t.corners()[i] : Double.NaN);
                out.add(t.decisionMargin());
                out.add(t.edgePx());
                out.add(t.undistortPx());
                out.add(t.reprojBestPx());
                out.add(t.reprojAltPx());
            }
        }
        double[] a = new double[out.size()];
        for (int i = 0; i < a.length; i++) a[i] = out.get(i);
        return a;
    }

    /**
     * Decodes {@link #encode}'s output; frames get {@code camera} as their name. An empty or
     * malformed array gives no frames.
     */
    public static List<FrameObservation> decode(String camera, double[] a) {
        var frames = new ArrayList<FrameObservation>();
        if (a == null || a.length < 2 || (int) a[0] != VERSION) return frames;
        try {
            int[] i = {2};
            int n = (int) a[1];
            for (int f = 0; f < n; f++) {
                long seq = (long) a[i[0]++];
                double ts = a[i[0]++];
                long pong = (long) a[i[0]++];
                boolean multi = a[i[0]++] != 0;
                Transform3d mt = getTransform(a, i);
                double mtErr = a[i[0]++];
                int idCount = (int) a[i[0]++];
                var ids = new ArrayList<Integer>();
                for (int k = 0; k < idCount; k++) ids.add((int) a[i[0]++]);
                int tagCount = (int) a[i[0]++];
                if (i[0] + (long) tagCount * TAG_SIZE > a.length) return frames;
                var tags = new ArrayList<TagObservation>();
                for (int k = 0; k < tagCount; k++) {
                    int id = (int) a[i[0]++];
                    Transform3d best = getTransform(a, i);
                    Transform3d alt = getTransform(a, i);
                    double amb = a[i[0]++], yaw = a[i[0]++], pitch = a[i[0]++], area = a[i[0]++];
                    double[] corners = new double[8];
                    for (int c = 0; c < 8; c++) corners[c] = a[i[0]++];
                    tags.add(new TagObservation(id, best, alt, amb, yaw, pitch, area, corners,
                            a[i[0]++], a[i[0]++], a[i[0]++], a[i[0]++], a[i[0]++]));
                }
                frames.add(new FrameObservation(camera, seq, ts, pong, multi ? mt : null, mtErr, ids, tags));
            }
        } catch (ArrayIndexOutOfBoundsException e) {
            // truncated: the frames decoded so far
        }
        return frames;
    }

    private static void putTransform(List<Double> out, Transform3d t) {
        out.add(t.getX());
        out.add(t.getY());
        out.add(t.getZ());
        Quaternion q = t.getRotation().getQuaternion();
        out.add(q.getW());
        out.add(q.getX());
        out.add(q.getY());
        out.add(q.getZ());
    }

    private static Transform3d getTransform(double[] a, int[] i) {
        var tr = new Translation3d(a[i[0]], a[i[0] + 1], a[i[0] + 2]);
        var q = new Quaternion(a[i[0] + 3], a[i[0] + 4], a[i[0] + 5], a[i[0] + 6]);
        i[0] += 7;
        return new Transform3d(tr, new Rotation3d(q));
    }
}
