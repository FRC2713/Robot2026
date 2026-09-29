// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc2713.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc2713.robot.FieldConstants;

public class VisionConstants {
  // Use the same 2026 field variant as the rest of the robot. Upload this layout to each camera.
  public static final AprilTagFieldLayout aprilTagLayout =
      FieldConstants.defaultAprilTagType.getLayout();

  // Must match the nickname configured in the PhotonVision web UI.
  public static final String camera0Name = "camera_0";

  // TODO: Replace the mounting dimensions with measured extrinsics.
  // WPILib robot frame: +X forward, +Y left, +Z up, angles in radians.
  public static final Transform3d robotToCamera0 =
      new Transform3d(
          Units.inchesToMeters(12.564),
          Units.inchesToMeters(8.0),
          Units.inchesToMeters(7.523),
          new Rotation3d(0.0, Units.degreesToRadians(-20.0), 0.0));

  public static final double maxAmbiguity = 0.3;
  public static final double maxZError = 0.75;
  public static final double maxSingleTagDistanceMeters = 3.0;

  // Standard deviations at 1 meter and 1 tag, scaled by distance squared / tag count.
  public static final double linearStdDevBaseline = 0.02;
  public static final double angularStdDevBaseline = 0.06;
  public static final boolean useVisionRotation = true;
  public static final boolean useVisionRotationSingleTag = false;
  public static final double[] cameraStdDevFactors = {1.0};
}
