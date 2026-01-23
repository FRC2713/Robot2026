package frc2713.lib.io;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ArticulatedComponentPublisher extends SubsystemBase {
  private ArticulatedComponent[] components;
  private AdvantageScopePathBuilder pb = new AdvantageScopePathBuilder("Robot");

  public ArticulatedComponentPublisher(ArticulatedComponent... articulatedComponents) {
    components = articulatedComponents;
  }

  public void publish() {
    publishCurrentTransforms();
    publishZeroedTransforms();
  }

  private void publishCurrentTransforms() {

    Transform3d[] transforms = new Transform3d[components.length];
    for (ArticulatedComponent ac : components) {
      int parentIndex = ac.getParentModelIndex();
      if (ac.getParentModelIndex() == -1) {
        transforms[ac.getModelIndex()] = ac.getTransform3d();
      } else {
        transforms[ac.getModelIndex()] = transforms[parentIndex].plus(ac.getTransform3d());
      }
    }

    Logger.recordOutput(pb.makePath("components", "transforms"), transforms);
  }

  private void publishZeroedTransforms() {

    Pose3d[] transforms = new Pose3d[components.length];
    for (ArticulatedComponent articulatedComponent : components) {
      transforms[articulatedComponent.getModelIndex()] = new Pose3d();
    }

    Logger.recordOutput(pb.makePath("components", "zeroed_transforms"), transforms);
  }

  @Override
  public void periodic() {
    publish();
  }
}
