// Copyright 2025-2026 FRC 8626
// https://github.com/team8626
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

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class VisionConstants {
  // Standard deviation baselines for 1 meter distance to single tag
  public static final double[] LINEAR_STD_DEV_BASELINES = {0.2, 0.2, 0.07, 0.3, 0.3, 0.3}; // Meters
  public static final double ANGULAR_STD_DEV_BASELINE = 1.0; // Radians

  // Rear left (8626CAM1), rear right (8626CAM2), front left (8626CAM3) camera names (must match
  // PhotonVision config)
  public static final String[] CAMERA_NAMES = {"8626CAM1", "8626CAM2", "8626CAM3"};

  public static final double MAX_AMBIGUITY = 0.3;
  public static final Distance MAX_Z_HEIGHT = Meters.of(0.2);

  public static final AprilTagFieldLayout APRIL_TAGS;

  static {
    AprilTagFieldLayout tryAprilTags;
    try {
      tryAprilTags =
          new AprilTagFieldLayout(Filesystem.getDeployDirectory().toPath() + "/apriltags.json");
      Logger.recordOutput("AprilTagLayoutType", "Custom");
    } catch (IOException e) {
      tryAprilTags = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
      Logger.recordOutput("AprilTagLayoutType", "Andymark");
    }
    APRIL_TAGS = tryAprilTags;
  }

  public static final Set<Integer> BLUE_HUB_TAG_IDS = Set.of(18, 19, 20, 21, 24, 25, 26, 27);
  public static final Set<Integer> RED_HUB_TAG_IDS = Set.of(2, 3, 4, 5, 8, 9, 10, 1);
  public static final Set<Integer> BLUE_CLIMB_TAG_IDS = Set.of(31, 32);
  public static final Set<Integer> RED_CLIMB_TAG_IDS = Set.of(15, 16);

  public static final double HUB_TAG_STD_DEV_BIAS = 0.08; // added to non-hub tags in hub mode
  public static final double CLIMB_TAG_STD_DEV_BIAS = 0.08; // added to non-climb tags in climb mode

  // Transforms from robot to cameras, (x forward, y left, z up), (roll, pitch,
  // yaw)
  public static final Transform3d[] CAMERA_TRANSFORMS = {

    // Rear left camera
    new Transform3d(
        new Translation3d(-0.332, 0.3175, 0.367),
        new Rotation3d(Degrees.of(3.195), Degrees.of(-19.688), Degrees.of(170.019))),

    // Rear right camera
    new Transform3d(
        new Translation3d(-0.332, -0.3175, 0.367),
        new Rotation3d(Degrees.of(-3.195), Degrees.of(-19.688), Degrees.of(-170.019))),

    // Front left camera
    new Transform3d(
        new Translation3d(0.332, 0.3175, 0.545),
        new Rotation3d(Degrees.of(-3.195), Degrees.of(-19.688), Degrees.of(9.981)))
  };
}
