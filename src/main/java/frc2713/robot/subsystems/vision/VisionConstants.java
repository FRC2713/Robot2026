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

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc2713.robot.FieldConstants;
import java.util.List;

public final class VisionConstants {
  private VisionConstants() {}

  public static final AprilTagFieldLayout aprilTagLayout =
      FieldConstants.defaultAprilTagType.getLayout();

  public record CameraConfig(String name, Transform3d robotToCamera, double stdDevFactor) {
    public CameraConfig {
      if (name == null
          || name.isBlank()
          || robotToCamera == null
          || !Double.isFinite(stdDevFactor)
          || stdDevFactor <= 0) {
        throw new IllegalArgumentException("Invalid vision camera configuration");
      }
    }
  }

  // Names must match PhotonVision exactly.
  // Fixed robot-to-camera mounts in WPILib coordinates: +X forward, +Y left, +Z up (meters).
  // Rotation3d takes roll, pitch, yaw; negative pitch aims up, positive yaw turns left.
  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html
  // The old SLAMDUNK robot frame had +Y forward and +X right: (x, y, z) -> (y, -x, z).
  // Rotations also account for the optical camera frame (+Z out, +X right, +Y down) becoming
  // PhotonVision's +X out, +Y left, +Z up. The right and back cameras retain their 180-degree roll.
  public static final List<CameraConfig> CAMERAS =
      List.of(
          new CameraConfig(
              "front_1",
              new Transform3d(
                  0.302558,
                  -0.063500,
                  0.502178,
                  new Rotation3d(Degrees.of(0), Degrees.of(-10), Degrees.of(0))),
              1.0),
          new CameraConfig(
              "right_2",
              new Transform3d(
                  -0.149225,
                  -0.362883,
                  0.518053,
                  new Rotation3d(Degrees.of(180), Degrees.of(-17.5), Degrees.of(-90))),
              1.0),
          new CameraConfig(
              "back_3",
              new Transform3d(
                  -0.313595,
                  0.0,
                  0.516118,
                  new Rotation3d(Degrees.of(180), Degrees.of(-17.5), Degrees.of(180))),
              1.0),
          new CameraConfig(
              "left_4",
              new Transform3d(
                  -0.149225,
                  0.362883,
                  0.518053,
                  new Rotation3d(Degrees.of(0), Degrees.of(-10), Degrees.of(90))),
              1.0));

  public static final double maxAmbiguity = 0.3;
  public static final double maxZError = 0.75;
  public static final double maxTiltRadians = Math.toRadians(20);
  public static final double maxSingleTagDistanceMeters = 3.0;
  public static final double maxMeasurementAgeSeconds = 0.5;
  public static final double futureTimestampToleranceSeconds = 0.05;
  public static final double resetMaxAgeSeconds = 0.25;

  // At 1 meter and 1 tag; scale by distance squared / tag count, with an accuracy floor.
  public static final double linearStdDevBaseline = 0.02;
  public static final double angularStdDevBaseline = 0.06;
  public static final double minLinearStdDev = 0.02;
  public static final double minAngularStdDev = Math.toRadians(1.5);
  public static final double ignoredAngularStdDev = 1e6;
  public static final boolean useVisionRotation = true;
  public static final boolean useVisionRotationSingleTag = false;

  public static final boolean REWIND_ENABLED = true;
  public static final double REWIND_DISABLED_TIMEOUT_SECONDS = 10.0;
  // In SIM mode, set true only when testing with a physical Jetson connected to the laptop.
  public static final boolean USE_PHYSICAL_CAMERAS_IN_SIM = false;
}
