package frc2713.lib.io;

import edu.wpi.first.math.geometry.Transform3d;

public interface ArticulatedComponent {
  public Transform3d getTransform3d();

  public int getModelIndex();

  public int getParentModelIndex();
}
