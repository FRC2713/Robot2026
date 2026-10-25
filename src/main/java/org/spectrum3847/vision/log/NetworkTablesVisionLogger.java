package org.spectrum3847.vision.log;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import org.spectrum3847.vision.VisionUpdate;

/**
 * A compact per-loop summary on NetworkTables, under {@code /<root>/} (default "SpectrumVision"):
 *
 * <ul>
 *   <li>{@code acceptedPoses}, {@code rejectedPoses}: Pose2d[] (AdvantageScope draws them on the
 *       field: green accepted, red rejected, as you configure it)
 *   <li>{@code acceptedStdDevs}: x, y, heading per accepted pose, flattened
 *   <li>{@code tagPoses}: Pose3d[] of the field tags behind this loop's accepted poses
 *   <li>{@code lastRejection}: "camera: gate: reason"
 *   <li>{@code <camera>/frames}, {@code accepted}, {@code rejected}: totals
 *   <li>{@code summary}: {@link VisionStats#summary()}, once a second
 * </ul>
 *
 * <p>All are "latest value" topics: dashboards that sample them don't pull every frame over the
 * radio (README, bandwidth).
 */
public class NetworkTablesVisionLogger implements VisionLogger {
    private final NetworkTable table;
    private final StructArrayPublisher<Pose2d> accepted, rejected;
    private final StructArrayPublisher<Pose3d> tagPoses;
    private final DoubleArrayPublisher acceptedStd;
    private final StringPublisher lastRejection, summary;
    private final Map<String, IntegerPublisher[]> perCamera = new HashMap<>();
    private final AprilTagFieldLayout field;
    private double lastSummary = -1e9;

    public NetworkTablesVisionLogger(String root, AprilTagFieldLayout field) {
        this.field = field;
        table = NetworkTableInstance.getDefault().getTable(root);
        accepted = table.getStructArrayTopic("acceptedPoses", Pose2d.struct).publish();
        rejected = table.getStructArrayTopic("rejectedPoses", Pose2d.struct).publish();
        tagPoses = table.getStructArrayTopic("tagPoses", Pose3d.struct).publish();
        acceptedStd = table.getDoubleArrayTopic("acceptedStdDevs").publish();
        lastRejection = table.getStringTopic("lastRejection").publish();
        summary = table.getStringTopic("summary").publish();
    }

    @Override
    public void log(VisionUpdate u, VisionStats stats) {
        accepted.set(u.acceptedPoses().toArray(new Pose2d[0]));
        rejected.set(u.rejectedPoses().toArray(new Pose2d[0]));
        double[] std = new double[u.accepted().size() * 3];
        var tags = new ArrayList<Pose3d>();
        var seen = new HashSet<Integer>();
        for (int i = 0; i < u.accepted().size(); i++) {
            var a = u.accepted().get(i);
            for (int k = 0; k < 3; k++) std[3 * i + k] = a.stdDevs().get(k, 0);
            for (int id : a.observation().tagIds()) {
                if (seen.add(id)) field.getTagPose(id).ifPresent(tags::add);
            }
        }
        acceptedStd.set(std);
        tagPoses.set(tags.toArray(new Pose3d[0]));
        if (!u.rejected().isEmpty()) {
            var r = u.rejected().get(u.rejected().size() - 1);
            lastRejection.set(r.observation().camera() + ": " + r.gate() + ": " + r.reason());
        }
        stats.cameras().forEach((name, c) -> {
            var p = perCamera.computeIfAbsent(name, n -> new IntegerPublisher[] {
                table.getSubTable(n).getIntegerTopic("frames").publish(),
                table.getSubTable(n).getIntegerTopic("accepted").publish(),
                table.getSubTable(n).getIntegerTopic("rejected").publish()
            });
            p[0].set(c.frames);
            p[1].set(c.accepted);
            p[2].set(c.rejected());
        });
        if (u.nowSeconds() - lastSummary >= 1.0) {
            lastSummary = u.nowSeconds();
            summary.set(stats.summary());
        }
    }
}
