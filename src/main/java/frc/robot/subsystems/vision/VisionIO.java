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

import org.littletonrobotics.junction.AutoLog;

/** Vision subsystem hardware interface. */
public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    // Connection status per camera
    public boolean[] connected = new boolean[] {};
    public String[] cameraNames = new String[] {};

    // Per-camera detection data (arrays indexed by camera)
    public boolean[] hasTargets = new boolean[] {};
    public double[] timestampSeconds = new double[] {};

    // Pose estimation results (per camera)
    public double[] poseX = new double[] {}; // meters
    public double[] poseY = new double[] {}; // meters
    public double[] poseRotationDeg = new double[] {}; // degrees
    public double[] poseAmbiguity = new double[] {}; // 0-1, lower is better

    // Quality metrics (per camera)
    public int[] tagCount = new int[] {}; // number of tags in view
    public double[] avgTagDistance = new double[] {}; // average distance to tags (meters)

    // Best target tracking (per camera)
    public int[] bestTargetId = new int[] {}; // -1 if none
    public double[] bestTargetAmbiguity = new double[] {};

    // Best target Transform3d (camera-to-target)
    public double[] bestTargetTransformX = new double[] {};
    public double[] bestTargetTransformY = new double[] {};
    public double[] bestTargetTransformZ = new double[] {};
    public double[] bestTargetRotationRoll = new double[] {};
    public double[] bestTargetRotationPitch = new double[] {};
    public double[] bestTargetRotationYaw = new double[] {};

    // Best target field-relative Pose3d
    public double[] bestTargetFieldX = new double[] {};
    public double[] bestTargetFieldY = new double[] {};
    public double[] bestTargetFieldZ = new double[] {};
    public double[] bestTargetFieldRotRoll = new double[] {};
    public double[] bestTargetFieldRotPitch = new double[] {};
    public double[] bestTargetFieldRotYaw = new double[] {};

    // All visible tag IDs across all cameras (for tracking)
    public int[] visibleTagIds = new int[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}
}
