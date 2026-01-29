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

import static frc.robot.subsystems.vision.VisionConstants.backCameraName;
import static frc.robot.subsystems.vision.VisionConstants.fieldLayout;
import static frc.robot.subsystems.vision.VisionConstants.frontCameraName;
import static frc.robot.subsystems.vision.VisionConstants.robotToBackCamera;
import static frc.robot.subsystems.vision.VisionConstants.robotToFrontCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** PhotonVision hardware implementation managing dual cameras. */
public class VisionIOPhotonVision implements VisionIO {
  private final PhotonCamera[] cameras;
  private final PhotonPoseEstimator[] poseEstimators;
  private final String[] cameraNames;

  public VisionIOPhotonVision() {
    // Initialize cameras array
    cameraNames = new String[] {frontCameraName, backCameraName};
    cameras = new PhotonCamera[2];
    poseEstimators = new PhotonPoseEstimator[2];

    // Front camera (index 0)
    cameras[0] = new PhotonCamera(frontCameraName);
    poseEstimators[0] =
        new PhotonPoseEstimator(
            fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToFrontCamera);
    poseEstimators[0].setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    // Back camera (index 1)
    cameras[1] = new PhotonCamera(backCameraName);
    poseEstimators[1] =
        new PhotonPoseEstimator(
            fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToBackCamera);
    poseEstimators[1].setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Initialize arrays
    inputs.cameraNames = cameraNames;
    inputs.connected = new boolean[cameras.length];
    inputs.hasTargets = new boolean[cameras.length];
    inputs.timestampSeconds = new double[cameras.length];
    inputs.poseX = new double[cameras.length];
    inputs.poseY = new double[cameras.length];
    inputs.poseRotationDeg = new double[cameras.length];
    inputs.poseAmbiguity = new double[cameras.length];
    inputs.tagCount = new int[cameras.length];
    inputs.avgTagDistance = new double[cameras.length];

    // Initialize best target arrays
    inputs.bestTargetId = new int[cameras.length];
    inputs.bestTargetAmbiguity = new double[cameras.length];
    inputs.bestTargetTransformX = new double[cameras.length];
    inputs.bestTargetTransformY = new double[cameras.length];
    inputs.bestTargetTransformZ = new double[cameras.length];
    inputs.bestTargetRotationRoll = new double[cameras.length];
    inputs.bestTargetRotationPitch = new double[cameras.length];
    inputs.bestTargetRotationYaw = new double[cameras.length];
    inputs.bestTargetFieldX = new double[cameras.length];
    inputs.bestTargetFieldY = new double[cameras.length];
    inputs.bestTargetFieldZ = new double[cameras.length];
    inputs.bestTargetFieldRotRoll = new double[cameras.length];
    inputs.bestTargetFieldRotPitch = new double[cameras.length];
    inputs.bestTargetFieldRotYaw = new double[cameras.length];

    for (int i = 0; i < cameras.length; i++) {
      inputs.bestTargetId[i] = -1; // No target by default
    }

    ArrayList<Integer> allVisibleTags = new ArrayList<>();

    // Process each camera
    for (int i = 0; i < cameras.length; i++) {
      PhotonPipelineResult result =
          cameras[i].getAllUnreadResults().isEmpty()
              ? new PhotonPipelineResult()
              : cameras[i].getAllUnreadResults().get(cameras[i].getAllUnreadResults().size() - 1);

      // Connection status (check if result is valid)
      inputs.connected[i] =
          result != null && (result.getTimestampSeconds() > 0 || result.hasTargets());

      if (!inputs.connected[i]) {
        continue;
      }

      inputs.hasTargets[i] = result.hasTargets();
      inputs.timestampSeconds[i] = result.getTimestampSeconds();

      if (result.hasTargets()) {
        // Collect tag IDs and calculate metrics
        var targets = result.getTargets();
        inputs.tagCount[i] = targets.size();

        double totalDistance = 0.0;
        for (PhotonTrackedTarget target : targets) {
          int fiducialId = target.getFiducialId();
          if (fiducialId >= 0 && !allVisibleTags.contains(fiducialId)) {
            allVisibleTags.add(fiducialId);
          }

          // Calculate distance to tag
          var transform = target.getBestCameraToTarget();
          totalDistance += transform.getTranslation().getNorm();
        }

        inputs.avgTagDistance[i] = targets.size() > 0 ? totalDistance / targets.size() : 0.0;

        // Get pose estimate from PhotonPoseEstimator
        Optional<org.photonvision.EstimatedRobotPose> poseResult = poseEstimators[i].update(result);

        if (poseResult.isPresent()) {
          var estimatedPose = poseResult.get();
          Pose2d pose2d = estimatedPose.estimatedPose.toPose2d();

          inputs.poseX[i] = pose2d.getX();
          inputs.poseY[i] = pose2d.getY();
          inputs.poseRotationDeg[i] = pose2d.getRotation().getDegrees();

          // Get ambiguity from best target (or 0 if multi-tag)
          inputs.poseAmbiguity[i] =
              targets.size() > 1
                  ? 0.0 // Multi-tag has no ambiguity metric
                  : result.getBestTarget().getPoseAmbiguity();
        }

        // Extract best target for auto-alignment
        PhotonTrackedTarget bestTarget = result.getBestTarget();
        if (bestTarget != null) {
          inputs.bestTargetId[i] = bestTarget.getFiducialId();
          inputs.bestTargetAmbiguity[i] = bestTarget.getPoseAmbiguity();

          // Camera-to-target transform
          var cameraToTarget = bestTarget.getBestCameraToTarget();
          inputs.bestTargetTransformX[i] = cameraToTarget.getTranslation().getX();
          inputs.bestTargetTransformY[i] = cameraToTarget.getTranslation().getY();
          inputs.bestTargetTransformZ[i] = cameraToTarget.getTranslation().getZ();
          inputs.bestTargetRotationRoll[i] = cameraToTarget.getRotation().getX();
          inputs.bestTargetRotationPitch[i] = cameraToTarget.getRotation().getY();
          inputs.bestTargetRotationYaw[i] = cameraToTarget.getRotation().getZ();

          // Field-relative pose from AprilTag layout
          Optional<Pose3d> tagPoseOpt = fieldLayout.getTagPose(bestTarget.getFiducialId());
          if (tagPoseOpt.isPresent()) {
            Pose3d tagFieldPose = tagPoseOpt.get();
            inputs.bestTargetFieldX[i] = tagFieldPose.getX();
            inputs.bestTargetFieldY[i] = tagFieldPose.getY();
            inputs.bestTargetFieldZ[i] = tagFieldPose.getZ();
            inputs.bestTargetFieldRotRoll[i] = tagFieldPose.getRotation().getX();
            inputs.bestTargetFieldRotPitch[i] = tagFieldPose.getRotation().getY();
            inputs.bestTargetFieldRotYaw[i] = tagFieldPose.getRotation().getZ();
          }
        }
      }
    }

    // Convert visible tags ArrayList to int array
    inputs.visibleTagIds = allVisibleTags.stream().mapToInt(Integer::intValue).toArray();
  }
}
