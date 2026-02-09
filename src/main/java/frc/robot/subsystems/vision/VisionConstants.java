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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.Set;

/** Constants for the vision subsystem. */
public class VisionConstants {
  // Camera identifiers (must match PhotonVision camera names)
  public static final String frontCameraName = "front_camera";
  public static final String backCameraName = "back_camera";

  // AprilTag field layout for 2026 Rebuilt
  public static final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

  // Camera mounting transforms (robot-relative)
  // 2026 KitBot square config: 26.5" x 26.5" (matches DriveConstants.wheelBase/trackWidth).
  // Front/back camera X = half wheelbase = 13.25" from center to front/back of frame.
  // WPILib: +X forward, +Y left, +Z up. Rotation: roll, pitch (negative = tilted up), yaw.
  public static final Transform3d robotToFrontCamera =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(13.25), // X: forward (half of 26.5" wheelbase)
              Units.inchesToMeters(0.0), // Y: left (centered)
              Units.inchesToMeters(24.0)), // Z: up (typical mast height)
          new Rotation3d(
              0.0, // Roll
              Units.degreesToRadians(-15.0), // Pitch (negative = tilted up for AprilTags)
              0.0)); // Yaw (facing forward)

  // Back camera: same X magnitude behind center, 180° yaw (facing backward)
  public static final Transform3d robotToBackCamera =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-13.25), // X: backward (half of 26.5" wheelbase)
              Units.inchesToMeters(0.0), // Y: left (centered)
              Units.inchesToMeters(24.0)), // Z: up (typical mast height)
          new Rotation3d(
              0.0, // Roll
              Units.degreesToRadians(-15.0), // Pitch (negative = tilted up)
              Units.degreesToRadians(180.0))); // Yaw (facing backward)

  // Quality filtering thresholds
  public static final double maxPoseAmbiguity = 0.2; // Reject poses with ambiguity > 0.2
  public static final double maxDistanceMeters = 5.0; // Reject tags farther than 5m
  public static final int minTagsForMultiTag = 2; // Require 2+ tags for multi-tag pose

  // Standard deviation calculation parameters
  // These tune how much we trust vision vs wheel odometry
  public static final double xyStdDevBase = 0.5; // Base std dev in meters
  public static final double thetaStdDevBase = 0.6; // Base std dev in radians
  public static final double xyStdDevPerMeter = 0.05; // Increase per meter of distance
  public static final double xyStdDevMultiTagDivisor = 2.0; // Divide by this for multi-tag

  // Simulation parameters
  public static final double simPoseNoiseStdDev = 0.01; // meters
  /** Max distance (m) between two tag positions on the field to treat as a multi-tag pair. */
  public static final double simMultiTagPairMaxDistanceMeters = 2.0;
  /** Pose noise std dev when multi-tag pair is visible (more accurate localization). */
  public static final double simPoseNoiseStdDevMultiTag = 0.005; // meters

  /** A pair of AprilTag IDs used for multi-tag lookup. Always stored with id1 &lt; id2. */
  public record TagPair(int id1, int id2) {
    public static TagPair of(int a, int b) {
      return new TagPair(Math.min(a, b), Math.max(a, b));
    }
  }

  /**
   * Lookup table of AprilTag ID pairs that are close together on the field. When a camera sees both
   * tags of a pair, the sim uses multi-tag localization.
   */
  public static final Set<TagPair> MULTI_TAG_PAIRS =
      Set.of(
          TagPair.of(15, 16),
          TagPair.of(13, 14),
          TagPair.of(5, 8),
          TagPair.of(9, 10),
          TagPair.of(2, 11),
          TagPair.of(3, 4),
          TagPair.of(19, 20),
          TagPair.of(18, 27),
          TagPair.of(25, 26),
          TagPair.of(21, 24),
          TagPair.of(31, 32),
          TagPair.of(29, 30));

  // Target scoring weights (must sum to 1.0)
  public static final double SCORE_WEIGHT_DISTANCE = 0.40;
  public static final double SCORE_WEIGHT_CENTEREDNESS = 0.30;
  public static final double SCORE_WEIGHT_AMBIGUITY = 0.20;
  public static final double SCORE_WEIGHT_HEADING_ALIGNMENT = 0.10;

  // Camera field of view parameters (approximate values for typical PhotonVision cameras)
  public static final double CAMERA_FOV_HORIZONTAL_DEG = 70.0;
  public static final double CAMERA_FOV_VERTICAL_DEG = 50.0;

  // Camera tilt angle (extracted from robotToFrontCamera/robotToBackCamera)
  public static final double CAMERA_TILT_DEG = 15.0;

  // Align-to-tag command: distance and control
  /** Target distance in meters from the tag when aligned (robot center to tag). */
  public static final double alignToTagDistanceMeters = 1.0;
  /**
   * Distance (meters) when aligning to a tower target. Used when the best target is 31/32 (align to
   * 31) or 15/16 (align to 15). 1.148m is the depth of the tower floor plate, plus offset.
   */
  public static final double alignToTowerDistanceMeters = 1.148 + 0.25;
  /**
   * Distance (meters) when aligning to a depot. Used when the best target is 29/30 (align to 29) or
   * 13/14 (align to 13). 0.5m is the offset from the depot.
   */
  public static final double alignToDepotDistanceMeters = 0.5;
  /** P gain for x/y position error (field-relative linear velocity). */
  public static final double alignLinearKp = 2.0;
  /** Max linear speed (m/s) when aligning, for smooth approach. */
  public static final double alignMaxLinearSpeedMetersPerSec = 2.0;
  /** Angular P gain for heading (ProfiledPIDController). */
  public static final double alignAngularKp = 5.0;
  /** Angular D gain for heading. */
  public static final double alignAngularKd = 0.4;
  /** Max angular velocity (rad/s) for alignment. */
  public static final double alignAngularMaxVelocity = 8.0;
  /** Max angular acceleration (rad/s^2) for alignment. */
  public static final double alignAngularMaxAcceleration = 20.0;
  /** Position tolerance in meters to consider "at goal". */
  public static final double alignToleranceMeters = 0.05;
  /** Angle tolerance in radians to consider "at goal". */
  public static final double alignToleranceRadians = Units.degreesToRadians(2.0);
}
