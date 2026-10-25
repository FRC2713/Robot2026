package org.spectrum3847.vision.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.spectrum3847.vision.VisionSystem;
import org.spectrum3847.vision.io.PhotonCameraIO;

/**
 * PhotonLib's simulation for every PhotonVision camera in a {@link VisionSystem}: in simulation the
 * cameras see the field's tags from the robot's simulated pose, through the same VisionSystem code
 * that runs on the robot.
 *
 * <pre>{@code
 * // robotInit, when RobotBase.isSimulation():
 * visionSim = VisionSim.attach(vision, VisionSim.thriftiestCam());
 * // simulationPeriodic():
 * visionSim.update(drivetrainSim.getPose());   // the true (simulated) pose, not the estimate
 * }</pre>
 *
 * PhotonLib's simulation needs OpenCV (WPILib's desktop simulation has it).
 */
public class VisionSim {
    private final VisionSystemSim sim;

    private VisionSim(VisionSystemSim sim) {
        this.sim = sim;
    }

    /** Properties like a Thriftiest Cam at 1280x800 (~70° horizontal FOV), 100 fps, ~15 ms latency. */
    public static SimCameraProperties thriftiestCam() {
        return new SimCameraProperties()
                .setCalibration(1280, 800, Rotation2d.fromDegrees(80))
                .setCalibError(0.35, 0.10)
                .setFPS(100)
                .setAvgLatencyMs(15)
                .setLatencyStdDevMs(3);
    }

    /** Every {@link PhotonCameraIO} camera of {@code vision}, simulated with {@code props}. */
    public static VisionSim attach(VisionSystem vision, SimCameraProperties props) {
        var sim = new VisionSystemSim("SpectrumVision");
        sim.addAprilTags(vision.field());
        for (var cam : vision.cameras()) {
            if (cam.io() instanceof PhotonCameraIO io) {
                var cs = new PhotonCameraSim(io.camera(), props);
                cs.enableDrawWireframe(false);
                // No video streams: they cost CPU and nothing here reads them. (Turn them on
                // yourself through visionSystemSim() to watch the simulated cameras.)
                cs.enableRawStream(false);
                cs.enableProcessedStream(false);
                sim.addCamera(cs, cam.robotToCamera());
            }
        }
        return new VisionSim(sim);
    }

    /** Every simulation loop, with the robot's true simulated pose. */
    public void update(Pose2d truePose) {
        sim.update(truePose);
    }

    public VisionSystemSim visionSystemSim() {
        return sim;
    }
}
