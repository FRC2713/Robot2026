package frc2713.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import frc2713.lib.util.LoggedTunableBoolean;

/**
 * VisionIOLocalNT
 *
 * <p>A VisionIO implementation for testing the full slamdunk pose pipeline on a bench setup,
 * without a roboRIO or radio. Your laptop will be the network tables server. The Jetson and your
 * laptop are connected directly over Ethernet.
 *
 * <p>The pose data can then be visualized in AdvantageScope via the simulation's NT4 server.
 *
 * <p>======= SETUP INSTRUCTIONS =======
 *
 * <p>STEP 1 — Set your laptop's Ethernet IP to fake the roboRio (10.27.13.2) On Mac: System
 * Settings → Network → LAN → Set your IP to Manual (e.g. IP address: 10.27.12.2, subnet
 * 255.255.255.0).
 *
 * <p>STEP 2 — Run the robot simulation on your laptop The simulation starts an NT4 server on your
 * laptop on port 5810 by default.
 *
 * <p>STEP 3 — Open AdvantageScope and connect to the simulation You should see NT keys populate,
 * including the slamdunk table once the Jetson connects.
 *
 * <p>STEP 4 Verify slamdunk connected In AdvantageScope's NT browser, look for:
 * /slamdunk/pose_robot (double array — the raw pose from TagSLAM) If it appears and is updating,
 * slamdunk is connected and publishing.
 *
 * <p>====================================
 */
public class VisionIOLocalNT implements VisionIO {

  private final DoubleArraySubscriber poseSub;

  // Same transform as VisionIOSLAMDunk — 90 degree rotation around Z to align
  // slamdunk's coordinate frame with WPILib's field coordinate system.
  private static final Transform3d SLAMDUNK_TRANSFORM =
      new Transform3d(new Translation3d(), new Rotation3d(0, 0, Math.PI / 2));

  private double lastTimestamp = -1;

  public LoggedTunableBoolean enableOffGroundFilter =
      new LoggedTunableBoolean("Vision/EnableOffGroundFilter", false);

  public VisionIOLocalNT() {
    var inst = NetworkTableInstance.getDefault();

    // Run as NT4 server so slamdunk (which runs as a client) connects to us.
    // In the real robot, VisionIOSLAMDunk uses the default instance which connects
    // as a client to the roboRIO's NT server. Here we flip the roles.
    // The simulation framework starts the server automatically, but calling
    // startServer() explicitly ensures it's up before slamdunk tries to connect.
    inst.startServer();

    var table = inst.getTable("slamdunk");
    poseSub = table.getDoubleArrayTopic("pose_robot").subscribe(new double[0]);

    System.out.println("[VisionIOLocalNT] NT4 server started.");
  }

  /**
   * pose_robot array layout published by slamdunk: [0] = timestamp (seconds, slamdunk clock)
   * [1,2,3] = translation x, y, z (meters) [4,5,6,7]= quaternion w, x, y, z [8] = tag count (int)
   * [9] = subgraph error [10] = average tag size (pixels)
   */
  @Override
  public void updateInputs(VisionInputs inputs) {
    // Reset pose to zero each cycle — only set it if we have a valid new update
    inputs.pose = new Pose2d();

    var poseArray = poseSub.get();

    if (poseArray.length > 10) {
      double t = poseArray[0];

      // Only process if this is a new timestamp
      if (lastTimestamp != t) {
        lastTimestamp = t;
        inputs.timestamp = t;
        inputs.tagCount = (int) poseArray[8];
        inputs.subgraphError = poseArray[9];
        inputs.avgTagSize = poseArray[10];

        // No FPGA clock in sim so latency isn't meaningful, set to zero
        inputs.latency = Seconds.of(0);

        var rawPose =
            new Pose3d(
                new Translation3d(poseArray[1], poseArray[2], poseArray[3]),
                new Rotation3d(
                    new Quaternion(poseArray[4], poseArray[5], poseArray[6], poseArray[7])));

        inputs.rawYaw = Rotation2d.fromRadians(rawPose.getRotation().getZ());
        inputs.pose3d = rawPose.transformBy(SLAMDUNK_TRANSFORM);
        inputs.pose = inputs.pose3d.toPose2d();

        if (enableOffGroundFilter.get() && Math.abs(inputs.pose3d.getTranslation().getZ()) > 0.1) {
          inputs.applying = false;
          inputs.reasoning = "Off ground Z > 0.1m";
          return;
        }

        inputs.applying = true;
        inputs.reasoning = "Valid pose";
      } else {
        inputs.applying = false;
        inputs.reasoning = "Stale timestamp";
      }
    } else {
      inputs.applying = false;
      inputs.reasoning =
          poseArray.length == 0 ? "No data" : "Incomplete pose array (n=" + poseArray.length + ")";
    }
  }

  @Override
  public void setGyroAngle(Angle angle) {
    // no-op in sim
  }
}
