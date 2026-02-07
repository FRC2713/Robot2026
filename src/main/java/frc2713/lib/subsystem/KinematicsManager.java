package frc2713.lib.subsystem;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2713.lib.io.AdvantageScopePathBuilder;
import frc2713.lib.io.ArticulatedComponent;
import frc2713.robot.FieldConstants;
import frc2713.robot.subsystems.drive.Drive;
import java.util.*;
import org.littletonrobotics.junction.Logger;

public class KinematicsManager extends SubsystemBase {
  private static KinematicsManager instance;

  // Data Classes
  private static class Node {
    final ArticulatedComponent component;
    final int id;
    final int parentId;
    final boolean publishable;

    public Node(ArticulatedComponent component, int id, int parentId, boolean publishable) {
      this.component = component;
      this.id = id;
      this.parentId = parentId;
      this.publishable = publishable;
    }
  }

  // State
  private final List<Node> nodes = new ArrayList<>();
  private Node[] orderedNodes = new Node[0]; // Cached sorted array
  private Pose3d[] globalPoses = new Pose3d[0];
  private Pose3d[] localPoses = new Pose3d[0];
  // Cache the IDs of nodes that need publishing so we don't search for them every loop
  private int[] publishableIndices = new int[0];

  // Cache the output array to avoid allocation
  private Pose3d[] mechanismPosesBuffer = new Pose3d[0];

  // Storage for velocities
  private Translation3d[] globalLinVels = new Translation3d[0];
  private Translation3d[] globalAngVels = new Translation3d[0];

  // Lookup map for O(1) access by Component instance
  private final Map<ArticulatedComponent, Integer> componentToIdMap = new HashMap<>();

  private final AdvantageScopePathBuilder pb = new AdvantageScopePathBuilder("Kinematics");
  private boolean isDirty = false; // Tracks if we need to re-sort

  public KinematicsManager() {
    if (instance != null) {
      throw new IllegalStateException("KinematicsManager already initialized!");
    }
    instance = this;
  }

  public static KinematicsManager getInstance() {
    // Lazy load logic could go here, but for Subsystems, constructor is better
    return instance;
  }

  /**
   * Registers a component into the kinematic chain. Call this in RobotContainer after creating the
   * subsystem. * @param component The subsystem/component instance
   *
   * @param id The unique ID for this node (e.g. 0 for Chassis)
   * @param parentId The ID of the parent node (-1 for Root)
   */
  public void register(ArticulatedComponent component, int id, int parentId) {
    nodes.add(new Node(component, id, parentId, true));
    componentToIdMap.put(component, id);
    isDirty = true; // Mark for re-sort on next loop
  }

  /**
   * Registers a component into the kinematic chain. Call this in RobotContainer after creating the
   * subsystem. * @param component The subsystem/component instance
   *
   * @param id The unique ID for this node (e.g. 0 for Chassis)
   * @param parentId The ID of the parent node (-1 for Root)
   */
  public void registerUnpublished(ArticulatedComponent component, int id, int parentId) {
    nodes.add(new Node(component, id, parentId, false));
    componentToIdMap.put(component, id);
    isDirty = true; // Mark for re-sort on next loop
  }

  @Override
  public void periodic() {
    // 1. Rebuild topology if new components were added
    if (isDirty) {
      rebuildTopology();
    }

    // 2. Update Kinematics
    updateKinematics();
    // Fast Copy: Only iterate the specific indices we care about
    for (int i = 0; i < publishableIndices.length; i++) {
      int nodeID = publishableIndices[i];
      mechanismPosesBuffer[i] = localPoses[nodeID];
    }

    // Log the pre-filled buffer
    if (mechanismPosesBuffer.length > 0) {
      Logger.recordOutput(pb.makePath("mechanismPoses"), mechanismPosesBuffer);
    }
    Logger.recordOutput(pb.makePath("localPoses"), localPoses);
    Logger.recordOutput(pb.makePath("globalPoses"), globalPoses);
  }

  private void rebuildTopology() {
    // Find max ID to size arrays
    int maxId = nodes.stream().mapToInt(n -> n.id).max().orElse(0);

    this.globalPoses = new Pose3d[maxId + 1];
    this.localPoses = new Pose3d[maxId + 1];
    Arrays.fill(this.globalPoses, new Pose3d());
    Arrays.fill(this.localPoses, new Pose3d());

    // Topological Sort
    // We sort the 'nodes' list so parents appear before children
    List<Node> sorted = new ArrayList<>();
    List<Node> remaining = new ArrayList<>(nodes);

    boolean progress = true;
    while (!remaining.isEmpty() && progress) {
      progress = false;
      for (int i = 0; i < remaining.size(); i++) {
        Node current = remaining.get(i);

        // Check if parent is processed
        boolean parentProcessed =
            (current.parentId == -1) || sorted.stream().anyMatch(n -> n.id == current.parentId);

        if (parentProcessed) {
          sorted.add(current);
          remaining.remove(i);
          i--;
          progress = true;
        }
      }
    }

    // Add any stragglers (cycles/missing parents) to prevent crash
    sorted.addAll(remaining);

    this.orderedNodes = sorted.toArray(new Node[0]);

    // Resize velocity arrays
    int size = globalPoses.length;
    this.globalLinVels = new Translation3d[size];
    this.globalAngVels = new Translation3d[size];
    Arrays.fill(this.globalLinVels, new Translation3d());
    Arrays.fill(this.globalAngVels, new Translation3d());

    // 1. Filter nodes to find which ones are publishable
    List<Integer> indicesList = new ArrayList<>();
    for (Node node : orderedNodes) {
      if (node.publishable) {
        indicesList.add(node.id);
      }
    }

    // 2. specific optimization: Pre-allocate the primitive arrays once
    this.publishableIndices = indicesList.stream().mapToInt(i -> i).toArray();
    this.mechanismPosesBuffer = new Pose3d[this.publishableIndices.length];

    // Initialize buffer to prevent nulls if logged before first update
    Arrays.fill(this.mechanismPosesBuffer, new Pose3d());

    this.isDirty = false;
  }

  private void updateKinematics() {
    for (Node node : orderedNodes) {
      Transform3d transform = node.component.getTransform3d();

      // Get local velocities and rotate them into the GLOBAL frame immediately
      // This simplifies the math: Global = Parent_Global + Local_Rotated_To_Global
      Rotation3d globalRot =
          (node.parentId == -1)
              ? transform.getRotation()
              : globalPoses[node.parentId].getRotation().plus(transform.getRotation());

      // Rotate local vectors to global frame
      Translation3d localLinDelta = node.component.getRelativeLinearVelocity().rotateBy(globalRot);
      Translation3d localAngDelta = node.component.getRelativeAngularVelocity().rotateBy(globalRot);

      if (node.parentId == -1) {
        // --- ROOT (Chassis) ---

        // Assume Root's "Relative" velocity is actually Field-Relative velocity
        globalPoses[node.id] = new Pose3d().plus(transform);
        localPoses[node.id] = new Pose3d();
        globalLinVels[node.id] = localLinDelta;
        globalAngVels[node.id] = localAngDelta;

      } else {
        // --- CHILD ---
        // 1. Update Pose
        globalPoses[node.id] = globalPoses[node.parentId].plus(transform);

        // This ensures the mechanism chain is built relative to the robot root
        if (node.parentId < localPoses.length) {
          localPoses[node.id] = localPoses[node.parentId].plus(transform);
        }

        // 2. Update Angular Velocity: Parent Global + Local Delta
        globalAngVels[node.id] = globalAngVels[node.parentId].plus(localAngDelta);

        // 3. Update Linear Velocity:
        // V_child = V_parent + (Omega_parent x Radius_vector) + V_local_delta

        // Radius vector: Vector from Parent Origin -> Child Origin (in Global Frame)
        Translation3d radius =
            globalPoses[node.id]
                .getTranslation()
                .minus(globalPoses[node.parentId].getTranslation());

        // Cross Product: Omega x Radius (Tangential Velocity)
        Translation3d tangentialVel = new Translation3d(globalAngVels[node.parentId].cross(radius));

        globalLinVels[node.id] =
            globalLinVels[node.parentId].plus(tangentialVel).plus(localLinDelta);
      }
    }
  }

  // --- API ---

  public Pose3d getGlobalPoseFor(ArticulatedComponent component) {
    Integer id = componentToIdMap.get(component);
    if (id == null) return new Pose3d(); // Not registered
    return getGlobalPose(id);
  }

  public Pose3d getGlobalPose(int id) {
    if (id < 0 || id >= globalPoses.length) return new Pose3d();
    return globalPoses[id];
  }

  // --- Helpers ---

  public Translation3d getGlobalLinearVelocity(ArticulatedComponent c) {
    Integer id = componentToIdMap.get(c);
    if (id == null || id >= globalLinVels.length) return new Translation3d();
    return globalLinVels[id];
  }

  public Translation3d getGlobalLinearVelocity(int index) {
    return globalLinVels[index];
  }

  public Rotation3d getGlobalAngularVelocity(int index) {
    return new Rotation3d(
        globalAngVels[index].getX(), globalAngVels[index].getY(), globalAngVels[index].getZ());
  }

  public Pose3d limitPoseToField(Pose3d pose) {
    double clampedX =
        Math.min(
            Math.max(Drive.DRIVE_BASE_RADIUS, pose.getTranslation().getX()),
            FieldConstants.fieldLength - Drive.DRIVE_BASE_RADIUS);
    double clampedY =
        Math.min(
            Math.max(Drive.DRIVE_BASE_RADIUS, pose.getTranslation().getY()),
            FieldConstants.fieldWidth - Drive.DRIVE_BASE_RADIUS);
    return new Pose3d(clampedX, clampedY, pose.getTranslation().getZ(), pose.getRotation());
  }
}
