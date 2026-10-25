package org.spectrum3847.vision.io;

import java.util.ArrayList;
import java.util.List;
import org.spectrum3847.vision.observation.FrameObservation;

/**
 * A camera fed by hand: tests, simulations of edge cases, and replaying logged frames. Frames
 * queued with {@link #queue} come out on the next {@link #update}.
 */
public class FakeCameraIO implements CameraIO {
    private final String name;
    private final List<FrameObservation> queued = new ArrayList<>();
    private boolean connected = true;

    public FakeCameraIO(String name) {
        this.name = name;
    }

    @Override
    public String name() {
        return name;
    }

    public synchronized FakeCameraIO queue(FrameObservation... frames) {
        queued.addAll(List.of(frames));
        return this;
    }

    public synchronized FakeCameraIO queue(List<FrameObservation> frames) {
        queued.addAll(frames);
        return this;
    }

    public synchronized void setConnected(boolean connected) {
        this.connected = connected;
    }

    @Override
    public synchronized void update(Inputs inputs) {
        inputs.connected = connected;
        inputs.frames = new ArrayList<>(queued);
        inputs.resultCount = queued.size();
        queued.clear();
    }
}
