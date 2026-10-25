package org.spectrum3847.vision.compat;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.HashMap;
import java.util.Map;
import org.photonvision.PhotonCamera;

/**
 * PhotonLib differences between the builds (WPILib 2026, PhotonLib 2026). PhotonLib 2026 has no
 * {@code PhotonCamera.setEnabled}; PhotonVision servers that support it (SpectrumJetson's, and
 * upstream from #2484) read the same topic PhotonLib 2027 writes, {@code enabledRequest}, so this
 * publishes it directly. Stock 2026 PhotonVision ignores it.
 */
public final class CameraCompat {
    private CameraCompat() {}

    private static final Map<String, BooleanPublisher> publishers = new HashMap<>();

    public static synchronized void requestEnabled(PhotonCamera camera, boolean enabled) {
        var pub =
                publishers.computeIfAbsent(
                        camera.getName(),
                        n -> NetworkTableInstance.getDefault()
                                .getTable("photonvision")
                                .getSubTable(n)
                                .getBooleanTopic("enabledRequest")
                                .publish());
        pub.set(enabled);
    }
}
