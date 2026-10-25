package org.spectrum3847.vision.io;

import java.util.ArrayList;
import java.util.List;
import org.spectrum3847.vision.observation.FrameObservation;
import org.spectrum3847.vision.observation.ObservationCodec;

/**
 * Where a camera's frames come from: the only part that touches NetworkTables. Everything after it
 * is deterministic, so with AdvantageKit you log {@link Inputs} and replay them (see the README's
 * AdvantageKit section); in tests, {@link FakeCameraIO} feeds frames by hand.
 */
public interface CameraIO {
    /** Fills {@code inputs} with everything new since the last call. Called once per robot loop. */
    void update(Inputs inputs);

    /** The camera's name (as in PhotonVision). */
    String name();

    /** What one update gives. Plain fields, so an AdvantageKit LoggableInputs can wrap it. */
    class Inputs {
        /** PhotonLib's view: results arriving and PhotonVision's heartbeat moving. */
        public boolean connected = false;
        /** New frames with at least one tag, oldest first. */
        public List<FrameObservation> frames = new ArrayList<>();
        /** Results received this update (with or without tags), for health. */
        public int resultCount = 0;
        /** Frames that got SpectrumJetson's per-tag quality (0 on stock PhotonVision). */
        public long qualityMatched = 0;

        /** The frames as one array (ObservationCodec), for logging. */
        public double[] framesAsArray() {
            return ObservationCodec.encode(frames);
        }

        /** Restores frames logged with {@link #framesAsArray()} (replay). */
        public void framesFromArray(String camera, double[] a) {
            frames = ObservationCodec.decode(camera, a);
        }
    }
}
