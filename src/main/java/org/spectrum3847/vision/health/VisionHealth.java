package org.spectrum3847.vision.health;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import java.util.LinkedHashMap;
import java.util.Map;
import org.photonvision.targeting.PhotonPipelineResult;
import org.spectrum3847.vision.compat.VisionAlert;
import org.spectrum3847.vision.compat.VisionAlert.Level;
import org.spectrum3847.vision.io.CameraIO;
import org.spectrum3847.vision.log.VisionStats;

/**
 * Dashboard alerts for what the drive team must know before (and during) a match. Works with any
 * PhotonVision; the Jetson checks only fire when SpectrumJetson's topics exist.
 *
 * <ul>
 *   <li>a camera disconnected (after a 10 s start-up grace)
 *   <li>PhotonVision's message format differs from this robot's PhotonLib (the coprocessor's
 *       {@code rawBytes} type string against PhotonLib's): results can't be read
 *   <li>the coprocessor's time sync is stale (frames' {@code timeSinceLastPong})
 *   <li>a camera's fps is below {@link #minFps} while enabled (SpectrumJetson's {@code health/fps})
 *   <li>a camera reports a problem (SpectrumJetson's {@code health/problem}: stuck, wrong mode, ...)
 *   <li>the Jetson is throttling (over-current or temperature) or hot ({@link #maxJetsonTempC})
 *   <li>the Jetson is in quiet mode while the robot is enabled (it shouldn't be)
 *   <li>frames arrive but every candidate is rejected (with the top reason), while enabled
 * </ul>
 */
public class VisionHealth {
    public double minFps = 20;
    public double maxJetsonTempC = 92;
    public double startupGraceSeconds = 10;
    public double allRejectedAfterSeconds = 3;

    private final NetworkTable pv = NetworkTableInstance.getDefault().getTable("photonvision");
    private final Map<String, CameraAlerts> cameras = new LinkedHashMap<>();
    private final StringSubscriber throttle = pv.getSubTable("jetson").getStringTopic("throttle").subscribe("");
    private final DoubleSubscriber tjTemp = pv.getSubTable("jetson").getDoubleTopic("tjTempC").subscribe(Double.NaN);
    private final BooleanSubscriber quietNow =
            pv.getSubTable("jetson").getBooleanTopic("quietNow").subscribe(false);
    private final VisionAlert throttleAlert = new VisionAlert("Vision", "", Level.WARNING);
    private final VisionAlert hotAlert = new VisionAlert("Vision", "", Level.WARNING);
    private final VisionAlert quietAlert =
            new VisionAlert("Vision", "Jetson in quiet mode while enabled (it should leave it within 0.1 s)", Level.WARNING);
    private final VisionAlert allRejected = new VisionAlert("Vision", "", Level.WARNING);
    private final double startedSeconds;
    private double rejectingSince = Double.NaN;

    private static final class CameraAlerts {
        final VisionAlert disconnected, format, sync, fps, problem;
        final DoubleSubscriber fpsSub;
        final StringSubscriber problemSub;
        final String name;

        CameraAlerts(NetworkTable pv, String name) {
            this.name = name;
            disconnected = new VisionAlert("Vision", "Camera " + name + " disconnected", Level.ERROR);
            format = new VisionAlert("Vision", "", Level.ERROR);
            sync = new VisionAlert("Vision", "Camera " + name + ": time sync stale (timestamps may be wrong)", Level.WARNING);
            fps = new VisionAlert("Vision", "", Level.WARNING);
            problem = new VisionAlert("Vision", "", Level.ERROR);
            fpsSub = pv.getSubTable(name).getSubTable("health").getDoubleTopic("fps").subscribe(Double.NaN);
            problemSub = pv.getSubTable(name).getSubTable("health").getStringTopic("problem").subscribe("");
        }
    }

    public VisionHealth(double nowSeconds) {
        startedSeconds = nowSeconds;
    }

    /** Once per loop, after the cameras' inputs are updated. */
    public void update(
            double now, boolean enabled, Map<String, CameraIO.Inputs> inputs, VisionStats.Counts loopCounts, String topReason) {
        boolean pastGrace = now - startedSeconds > startupGraceSeconds;
        inputs.forEach((name, in) -> {
            var c = cameras.computeIfAbsent(name, n -> new CameraAlerts(pv, n));
            c.disconnected.set(pastGrace && !in.connected);
            // The coprocessor's message format, from the rawBytes topic's type string.
            String remote = pv.getSubTable(name).getRawTopic("rawBytes").getTypeString();
            String local = PhotonPipelineResult.photonStruct.getTypeString();
            boolean mismatch = remote != null && !remote.isEmpty() && !remote.equals(local);
            c.format.setText("Camera " + name + ": PhotonVision's message format (" + remote
                    + ") isn't this PhotonLib's (" + local + "): update one to match the other");
            c.format.set(mismatch);
            long worstPong = -1;
            for (var f : in.frames) worstPong = Math.max(worstPong, f.timeSinceLastPongMicros());
            if (!in.frames.isEmpty()) c.sync.set(worstPong > 1_000_000);
            double fps = c.fpsSub.get();
            c.fps.setText(String.format("Camera %s: %.0f fps (below %.0f)", name, fps, minFps));
            c.fps.set(enabled && !Double.isNaN(fps) && fps < minFps);
            String p = c.problemSub.get();
            c.problem.setText("Camera " + name + ": " + p);
            c.problem.set(p != null && !p.isEmpty());
        });
        String t = throttle.get();
        throttleAlert.setText("Jetson throttling: " + t);
        // SpectrumJetson's reason: "None", "Prev. over-current (n)" (earlier, not now), or what's
        // limiting it now ("OVER-CURRENT", "HIGH TEMP (...)", "CPU CLOCK CAPPED", ...).
        throttleAlert.set(t != null && !t.isEmpty() && !t.equalsIgnoreCase("none") && !t.startsWith("Prev."));
        double temp = tjTemp.get();
        hotAlert.setText(String.format("Jetson at %.0f °C (cameras capped from 95 °C)", temp));
        hotAlert.set(!Double.isNaN(temp) && temp >= maxJetsonTempC);
        quietAlert.set(enabled && quietNow.get());
        // Frames arriving but nothing accepted, for a while: say why.
        if (enabled && loopCounts.candidates > 0 && loopCounts.accepted == 0) {
            if (Double.isNaN(rejectingSince)) rejectingSince = now;
        } else if (loopCounts.accepted > 0 || !enabled) {
            rejectingSince = Double.NaN;
        }
        boolean stuck = !Double.isNaN(rejectingSince) && now - rejectingSince > allRejectedAfterSeconds;
        allRejected.setText("Vision rejecting every pose (most: " + topReason + ")");
        allRejected.set(stuck);
    }
}
