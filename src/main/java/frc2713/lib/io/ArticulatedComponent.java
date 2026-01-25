package frc2713.lib.io;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc2713.lib.subsystem.KinematicsManager;

public interface ArticulatedComponent {
  // Required implementations
  public Transform3d getTransform3d();

  public int getModelIndex();

  public int getParentModelIndex();

  // --- New Helper Methods ---

  /** Gets the robot-relative pose of THIS component. */
  default Pose3d getGlobalPose() {
    if (KinematicsManager.getInstance() == null) {
      return new Pose3d();
    }
    return KinematicsManager.getInstance().getGlobalPose(this.getModelIndex());
  }

  /**
   * Calculates the transform required to move FROM this component TO the target. Example:
   * camera.getTransformTo(target)
   */
  default Transform3d getTransformTo(ArticulatedComponent target) {
    if (KinematicsManager.getInstance() == null) {
      return new Transform3d();
    }

    Pose3d myPose = this.getGlobalPose();
    Pose3d targetPose = target.getGlobalPose();

    // The transform that maps "My Pose" -> "Target Pose"
    return targetPose.minus(myPose);
  }

  default Transform3d getTransformTo(Pose3d pose) {
    if (KinematicsManager.getInstance() == null) {
      return new Transform3d();
    }

    Pose3d myPose = this.getGlobalPose();
    return pose.minus(myPose);
  }

  /** Calculates the distance (norm) between this component and the target. */
  default double getDistanceTo(ArticulatedComponent target) {
    return getTransformTo(target).getTranslation().getNorm();
  }
}
