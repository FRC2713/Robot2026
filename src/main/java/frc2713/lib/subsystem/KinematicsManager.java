package frc2713.lib.subsystem;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2713.lib.io.AdvantageScopePathBuilder;
import frc2713.lib.io.ArticulatedComponent;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class KinematicsManager extends SubsystemBase {
  // 1. Static instance for global access
  private static KinematicsManager instance;

  private final ArticulatedComponent[] orderedComponents;
  private final Pose3d[] globalPoses;
  private final Pose3d[] localPoses;
  private final AdvantageScopePathBuilder pb = new AdvantageScopePathBuilder("Robot");

  public KinematicsManager(ArticulatedComponent... articulatedComponents) {
    // 2. Set the singleton instance
    instance = this;

    int maxIndex = 0;
    for (ArticulatedComponent ac : articulatedComponents) {
      maxIndex = Math.max(maxIndex, ac.getModelIndex());
    }

    // Safety: +1 size to handle 0-based indexing
    this.globalPoses = new Pose3d[maxIndex + 1];
    Arrays.fill(this.globalPoses, new Pose3d());
    this.localPoses = new Pose3d[maxIndex + 1];
    Arrays.fill(this.localPoses, new Pose3d());

    this.orderedComponents = topologicalSort(articulatedComponents);
  }

  /** Static accessor for the interface to use. */
  public static KinematicsManager getInstance() {
    return instance;
  }

  /* ... topologicalSort method (same as previous) ... */
  private ArticulatedComponent[] topologicalSort(ArticulatedComponent[] input) {
    // (Include the topological sort logic from the previous response here)
    // Condensed for brevity in this snippet
    List<ArticulatedComponent> sorted = new ArrayList<>();
    List<ArticulatedComponent> remaining = new ArrayList<>(Arrays.asList(input));
    boolean progress = true;
    while (!remaining.isEmpty() && progress) {
      progress = false;
      for (int i = 0; i < remaining.size(); i++) {
        ArticulatedComponent current = remaining.get(i);
        int parentId = current.getParentModelIndex();
        boolean parentIsProcessed =
            (parentId == -1) || sorted.stream().anyMatch(c -> c.getModelIndex() == parentId);
        if (parentIsProcessed) {
          sorted.add(current);
          remaining.remove(i);
          i--;
          progress = true;
        }
      }
    }
    if (!remaining.isEmpty()) sorted.addAll(remaining);
    return sorted.toArray(new ArticulatedComponent[0]);
  }

  @Override
  public void periodic() {
    updateKinematics();
    Pose3d[] mechanismPoses = Arrays.copyOfRange(localPoses, 1, localPoses.length);
    Logger.recordOutput(pb.makePath("components", "transforms"), mechanismPoses);
  }

  private void updateKinematics() {
    for (ArticulatedComponent ac : orderedComponents) {
      int myIndex = ac.getModelIndex();
      int parentIndex = ac.getParentModelIndex();
      Transform3d subsystemTransform = ac.getTransform3d();

      if (parentIndex == -1) {
        // --- ROOT COMPONENT (Chassis / Index 0) ---

        // Global: Relative to Field (Odometry)
        globalPoses[myIndex] = new Pose3d().plus(subsystemTransform);

        // Local: Relative to Robot (It IS the Robot, so it is Identity/Zero)
        localPoses[myIndex] = new Pose3d();

      } else {
        // --- CHILD COMPONENT ---

        // Safety check
        if (parentIndex < globalPoses.length) {
          // Global: Parent Global + Transform
          globalPoses[myIndex] = globalPoses[parentIndex].plus(subsystemTransform);

          // Local: Parent Local + Transform
          // This maintains the chain relative to the Chassis (Index 0)
          localPoses[myIndex] = localPoses[parentIndex].plus(subsystemTransform);
        }
      }
    }
  }

  // --- Lookup API ---

  public Pose3d getGlobalPose(int modelIndex) {
    if (modelIndex < 0 || modelIndex >= globalPoses.length) return new Pose3d();
    return globalPoses[modelIndex];
  }
}
