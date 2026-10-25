package org.spectrum3847.vision;

import edu.wpi.first.math.geometry.Transform3d;
import org.spectrum3847.vision.io.CameraIO;

/**
 * One camera in a {@link VisionSystem}: where its frames come from, where it sits on the robot, and
 * how much to trust it relative to the others.
 */
public final class VisionCamera {
    private final CameraIO io;
    private volatile Transform3d robotToCamera;
    private volatile double trustFactor;
    private volatile boolean enabled = true;
    final CameraIO.Inputs inputs = new CameraIO.Inputs();

    VisionCamera(CameraIO io, Transform3d robotToCamera, double trustFactor) {
        this.io = io;
        this.robotToCamera = robotToCamera;
        this.trustFactor = trustFactor;
    }

    public String name() {
        return io.name();
    }

    public CameraIO io() {
        return io;
    }

    /** The latest inputs (read-only use). */
    public CameraIO.Inputs inputs() {
        return inputs;
    }

    public Transform3d robotToCamera() {
        return robotToCamera;
    }

    /** A measured mount (field calibration, or a turret's current angle) can replace the CAD one. */
    public void setRobotToCamera(Transform3d t) {
        robotToCamera = t;
    }

    /** Multiplies this camera's standard deviations: 2 = trusted half as much. */
    public double trustFactor() {
        return trustFactor;
    }

    public void setTrustFactor(double f) {
        trustFactor = f;
    }

    public boolean enabled() {
        return enabled;
    }

    /** Off: its frames are read (so the queue doesn't fill) but not used. */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }
}
