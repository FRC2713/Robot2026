package frc2713.lib.io;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc2713.lib.subsystem.KinematicsManager;

public interface ArticulatedComponent {
  // Required implementations
  default Transform3d getTransform3d() {
    return new Transform3d();
  }
  ;

  /**
   * * The linear velocity of this component relative to its parent, in this component's LOCAL
   * frame. Example: Elevator moving up = VecBuilder.fill(0, 0, 0.5)
   */
  default Vector<N3> getRelativeLinearVelocity() {
    return VecBuilder.fill(0, 0, 0);
  }

  /**
   * * The angular velocity of this component relative to its parent, in this component's LOCAL
   * frame. Example: Turret spinning left = VecBuilder.fill(0, 0, 2.5)
   */
  default Vector<N3> getRelativeAngularVelocity() {
    return VecBuilder.fill(0, 0, 0);
  }

  // --- Helpers ---

  default Vector<N3> getGlobalLinearVelocity() {
    if (KinematicsManager.getInstance() == null) return VecBuilder.fill(0, 0, 0);
    return KinematicsManager.getInstance().getGlobalLinearVelocity(this);
  }

  // --- New Helper Methods ---

  /** Gets the robot-relative pose of THIS component. */
  default Pose3d getGlobalPose() {
    if (KinematicsManager.getInstance() == null) {
      return new Pose3d();
    }
    return KinematicsManager.getInstance().getGlobalPoseFor(this);
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

  /**
   * Calculates the rotation required for this component's +X axis to point directly at the target.
   * Useful for turrets and arm pivots.
   *
   * @param target The target pose in 3D space.
   * @return The absolute Rotation3d (Yaw/Pitch) needed to aim at the target.
   */
  default Rotation3d getAimingRotation(Pose3d target) {
    if (KinematicsManager.getInstance() == null) return new Rotation3d();

    Translation3d difference = target.getTranslation().minus(this.getGlobalPose().getTranslation());
    double distanceFlat = Math.hypot(difference.getX(), difference.getY());

    // Calculate Yaw (Z-axis rotation)
    double yaw = Math.atan2(difference.getY(), difference.getX());

    // Calculate Pitch (Y-axis rotation) - assumes +Pitch is aiming UP
    double pitch = Math.atan2(difference.getZ(), distanceFlat);

    return new Rotation3d(0, -pitch, yaw); // Roll is usually 0 for aiming
  }

  /** Overload for aiming at another component (e.g., Camera aiming at a Game Piece). */
  default Rotation3d getAimingRotation(ArticulatedComponent target) {
    return getAimingRotation(target.getGlobalPose());
  }

  /**
   * Checks if this component is currently pointing at the target within a tolerance.
   *
   * @param target The target component.
   * @param tolerance The acceptable error.
   * @return True if aligned.
   */
  default boolean isPointingAt(ArticulatedComponent target, Angle tolerance) {
    Rotation3d currentRot = this.getGlobalPose().getRotation();
    Rotation3d targetRot = getAimingRotation(target);

    // Compare Yaw (Z) logic for basic turret alignment
    double error = Math.abs(currentRot.getZ() - targetRot.getZ());
    // Normalize error to 0-180
    error = Math.IEEEremainder(error, 2 * Math.PI);

    return Radians.of(Math.abs(error)).lte(tolerance);
  }

  /** Overload to get distance to a static Pose3d (like the Hub center). */
  default Distance getDistanceTo(Pose3d target) {
    if (KinematicsManager.getInstance() == null) return Meters.of(0.0);
    return Meters.of(this.getGlobalPose().getTranslation().getDistance(target.getTranslation()));
  }

  /**
   * Gets the horizontal distance (XY plane only) to a target. Critical for shooting calculations
   * where height difference is handled separately.
   */
  default Distance getDistance2d(ArticulatedComponent target) {
    return getDistance2d(target.getGlobalPose());
  }

  default Distance getDistance2d(Pose3d target) {
    if (KinematicsManager.getInstance() == null) return Meters.of(0.0);
    Translation3d myTrans = this.getGlobalPose().getTranslation();
    Translation3d targetTrans = target.getTranslation();

    return Meters.of(
        Math.hypot(targetTrans.getX() - myTrans.getX(), targetTrans.getY() - myTrans.getY()));
  }

  /**
   * Takes a field-relative pose and converts it to be relative to THIS component. Example: "Where
   * is the Hub relative to the Camera?" (Result x=3 means 3m in front of lens)
   */
  default Pose3d toLocalFrame(Pose3d fieldPose) {
    if (KinematicsManager.getInstance() == null) return new Pose3d();

    // Returns the transform that maps THIS component -> Field Pose
    return fieldPose.relativeTo(this.getGlobalPose());
  }

  /**
   * Takes a pose that is relative to THIS component and converts it to Field Relative. Example:
   * Vision gives you a target at x=2m relative to camera. Where is it on the field?
   */
  default Pose3d toFieldFrame(Transform3d localTransform) {
    if (KinematicsManager.getInstance() == null) return new Pose3d();

    return this.getGlobalPose().plus(localTransform);
  }

  /** Returns a 3D Vector (x, y, z) pointing FROM this component TO the target. */
  default Vector<N3> getVectorTo(Pose3d target) {
    if (KinematicsManager.getInstance() == null) {
      return new Vector<>(N3.instance);
    }

    // Calculate difference using Translation3d first
    Translation3d diff = target.getTranslation().minus(this.getGlobalPose().getTranslation());

    // Convert to Vector<N3>
    return VecBuilder.fill(diff.getX(), diff.getY(), diff.getZ());
  }
}
