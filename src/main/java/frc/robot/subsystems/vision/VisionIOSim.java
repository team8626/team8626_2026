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

import static frc.robot.subsystems.vision.VisionConstants.CAMERA_FOV_HORIZONTAL_DEG;
import static frc.robot.subsystems.vision.VisionConstants.CAMERA_FOV_VERTICAL_DEG;
import static frc.robot.subsystems.vision.VisionConstants.CAMERA_TILT_DEG;
import static frc.robot.subsystems.vision.VisionConstants.MULTI_TAG_PAIRS;
import static frc.robot.subsystems.vision.VisionConstants.SCORE_WEIGHT_AMBIGUITY;
import static frc.robot.subsystems.vision.VisionConstants.SCORE_WEIGHT_CENTEREDNESS;
import static frc.robot.subsystems.vision.VisionConstants.SCORE_WEIGHT_DISTANCE;
import static frc.robot.subsystems.vision.VisionConstants.SCORE_WEIGHT_HEADING_ALIGNMENT;
import static frc.robot.subsystems.vision.VisionConstants.backCameraName;
import static frc.robot.subsystems.vision.VisionConstants.fieldLayout;
import static frc.robot.subsystems.vision.VisionConstants.frontCameraName;
import static frc.robot.subsystems.vision.VisionConstants.maxDistanceMeters;
import static frc.robot.subsystems.vision.VisionConstants.robotToBackCamera;
import static frc.robot.subsystems.vision.VisionConstants.robotToFrontCamera;
import static frc.robot.subsystems.vision.VisionConstants.simPoseNoiseStdDev;
import static frc.robot.subsystems.vision.VisionConstants.simPoseNoiseStdDevMultiTag;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionConstants.TagPair;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Simulation implementation for vision subsystem. */
public class VisionIOSim implements VisionIO {
  private final VisionSystemSim visionSim;
  private final PhotonCameraSim[] cameraSims;
  private final String[] cameraNames;
  private final Random random = new Random();

  private Pose2d robotPose = new Pose2d();

  /** Container for a scored target candidate. */
  private static class ScoredTarget {
    final PhotonTrackedTarget target;
    final int cameraIndex;
    final Transform3d cameraToTarget;
    final double score;
    final double distanceScore;
    final double centernessScore;
    final double ambiguityScore;

    ScoredTarget(
        PhotonTrackedTarget target,
        int cameraIndex,
        Transform3d cameraToTarget,
        double score,
        double distanceScore,
        double centernessScore,
        double ambiguityScore) {
      this.target = target;
      this.cameraIndex = cameraIndex;
      this.cameraToTarget = cameraToTarget;
      this.score = score;
      this.distanceScore = distanceScore;
      this.centernessScore = centernessScore;
      this.ambiguityScore = ambiguityScore;
    }
  }

  public VisionIOSim() {
    cameraNames = new String[] {frontCameraName, backCameraName};

    // Create vision system simulator
    visionSim = new VisionSystemSim("main");
    visionSim.addAprilTags(fieldLayout);

    // Create simulated cameras
    cameraSims = new PhotonCameraSim[2];

    // Front camera
    cameraSims[0] = new PhotonCameraSim(new PhotonCamera(frontCameraName));
    visionSim.addCamera(cameraSims[0], robotToFrontCamera);

    // Back camera
    cameraSims[1] = new PhotonCameraSim(new PhotonCamera(backCameraName));
    visionSim.addCamera(cameraSims[1], robotToBackCamera);
  }

  /**
   * Update simulation with robot pose from odometry. This should be called from Vision.periodic()
   * with current pose.
   */
  public void updateRobotPose(Pose2d pose) {
    robotPose = pose;
    visionSim.update(new Pose3d(pose));
  }

  /**
   * Calculate scoring for a target based on distance, centeredness, and ambiguity.
   *
   * @param target The PhotonTrackedTarget to score
   * @param cameraIndex Which camera sees this target (0=front, 1=back)
   * @param simulatedAmbiguity Ambiguity value for this target
   * @return ScoredTarget with computed score
   */
  private ScoredTarget scoreTarget(
      PhotonTrackedTarget target, int cameraIndex, double simulatedAmbiguity) {
    var cameraToTarget = target.getBestCameraToTarget();

    // 1. Distance score (closer is better)
    double distance = cameraToTarget.getTranslation().getNorm();
    double distanceScore = Math.max(0.0, 1.0 - (distance / maxDistanceMeters));

    // 2. Centeredness score (more centered in FOV is better)
    double centernessScore = calculateCenternessScore(cameraToTarget);

    // 3. Ambiguity score (lower ambiguity is better)
    double ambiguityScore = Math.max(0.0, 1.0 - simulatedAmbiguity);

    // 4. Heading alignment score (not available in VisionIO layer - use neutral value)
    double headingScore = 0.5; // Neutral when unavailable

    // Weighted total score
    double totalScore =
        (distanceScore * SCORE_WEIGHT_DISTANCE)
            + (centernessScore * SCORE_WEIGHT_CENTEREDNESS)
            + (ambiguityScore * SCORE_WEIGHT_AMBIGUITY)
            + (headingScore * SCORE_WEIGHT_HEADING_ALIGNMENT);

    return new ScoredTarget(
        target,
        cameraIndex,
        cameraToTarget,
        totalScore,
        distanceScore,
        centernessScore,
        ambiguityScore);
  }

  /**
   * Calculate how centered a target is in the camera's field of view.
   *
   * @param cameraToTarget Transform from camera to target
   * @return Score from 0.0 (edge/outside FOV) to 1.0 (perfectly centered)
   */
  private double calculateCenternessScore(Transform3d cameraToTarget) {
    // Camera frame: x=forward, y=left, z=up
    double x = cameraToTarget.getTranslation().getX();
    double y = cameraToTarget.getTranslation().getY();
    double z = cameraToTarget.getTranslation().getZ();

    // Horizontal deviation (yaw) - how far left/right from center
    double horizontalAngleDeg = Math.toDegrees(Math.atan2(y, x));
    double horizontalDeviation = Math.abs(horizontalAngleDeg);

    // Vertical deviation (pitch) - account for camera tilt
    double verticalAngleDeg = Math.toDegrees(Math.atan2(z, Math.sqrt(x * x + y * y)));
    double verticalDeviation = Math.abs(verticalAngleDeg - CAMERA_TILT_DEG);

    // Normalize by half FOV (max deviation to edge)
    double normalizedHorizontal = horizontalDeviation / (CAMERA_FOV_HORIZONTAL_DEG / 2.0);
    double normalizedVertical = verticalDeviation / (CAMERA_FOV_VERTICAL_DEG / 2.0);

    // Combined deviation using Euclidean distance
    double angularDeviation =
        Math.sqrt(
            normalizedHorizontal * normalizedHorizontal + normalizedVertical * normalizedVertical);

    // Score: 1.0 = centered, 0.0 = at edge or beyond
    return Math.max(0.0, 1.0 - angularDeviation);
  }

  /**
   * Returns true if at least two of the visible tags form a known multi-tag pair (see
   * VisionConstants.simMultiTagPairIds). O(1) lookup per pair.
   */
  private boolean hasMultiTagPair(List<PhotonTrackedTarget> targets) {
    if (targets.size() < 2) {
      return false;
    }
    int[] ids = new int[targets.size()];
    int n = 0;
    for (var target : targets) {
      int id = target.getFiducialId();
      if (id >= 0) {
        ids[n++] = id;
      }
    }
    for (int i = 0; i < n; i++) {
      for (int j = i + 1; j < n; j++) {
        int id1 = Math.min(ids[i], ids[j]);
        int id2 = Math.max(ids[i], ids[j]);
        if (MULTI_TAG_PAIRS.contains(TagPair.of(id1, id2))) {
          return true;
        }
      }
    }
    return false;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Initialize arrays
    inputs.cameraNames = cameraNames;
    inputs.connected = new boolean[cameraSims.length];
    inputs.hasTargets = new boolean[cameraSims.length];
    inputs.timestampSeconds = new double[cameraSims.length];
    inputs.poseX = new double[cameraSims.length];
    inputs.poseY = new double[cameraSims.length];
    inputs.poseRotationDeg = new double[cameraSims.length];
    inputs.poseAmbiguity = new double[cameraSims.length];
    inputs.tagCount = new int[cameraSims.length];
    inputs.avgTagDistance = new double[cameraSims.length];

    // Initialize best target arrays
    inputs.bestTargetId = new int[cameraSims.length];
    inputs.bestTargetAmbiguity = new double[cameraSims.length];
    inputs.bestTargetTransformX = new double[cameraSims.length];
    inputs.bestTargetTransformY = new double[cameraSims.length];
    inputs.bestTargetTransformZ = new double[cameraSims.length];
    inputs.bestTargetRotationRoll = new double[cameraSims.length];
    inputs.bestTargetRotationPitch = new double[cameraSims.length];
    inputs.bestTargetRotationYaw = new double[cameraSims.length];
    inputs.bestTargetFieldX = new double[cameraSims.length];
    inputs.bestTargetFieldY = new double[cameraSims.length];
    inputs.bestTargetFieldZ = new double[cameraSims.length];
    inputs.bestTargetFieldRotRoll = new double[cameraSims.length];
    inputs.bestTargetFieldRotPitch = new double[cameraSims.length];
    inputs.bestTargetFieldRotYaw = new double[cameraSims.length];

    for (int i = 0; i < cameraSims.length; i++) {
      inputs.bestTargetId[i] = -1; // No target by default
    }

    ArrayList<Integer> allVisibleTags = new ArrayList<>();

    // Process each camera
    for (int i = 0; i < cameraSims.length; i++) {
      var results = cameraSims[i].getCamera().getAllUnreadResults();

      inputs.connected[i] = true; // Always connected in sim

      if (results.isEmpty()) {
        continue;
      }

      var result = results.get(results.size() - 1);
      inputs.hasTargets[i] = result.hasTargets();
      inputs.timestampSeconds[i] = result.getTimestampSeconds();

      if (result.hasTargets()) {
        var targets = result.getTargets();
        inputs.tagCount[i] = targets.size();

        // Collect visible tag IDs
        for (var target : targets) {
          int fiducialId = target.getFiducialId();
          if (fiducialId >= 0 && !allVisibleTags.contains(fiducialId)) {
            allVisibleTags.add(fiducialId);
          }
        }

        // Multi-tag localization: when a close pair of AprilTags is visible, mimic
        // MULTI_TAG_PNP_ON_COPROCESSOR with lower pose noise (more accurate estimate).
        boolean multiTagPair = hasMultiTagPair(targets);
        double poseNoise = multiTagPair ? simPoseNoiseStdDevMultiTag : simPoseNoiseStdDev;
        double thetaNoiseDeg = multiTagPair ? 0.5 : 1.0; // degrees std dev

        double noiseX = random.nextGaussian() * poseNoise;
        double noiseY = random.nextGaussian() * poseNoise;
        double noiseTheta = random.nextGaussian() * Math.toRadians(thetaNoiseDeg);

        inputs.poseX[i] = robotPose.getX() + noiseX;
        inputs.poseY[i] = robotPose.getY() + noiseY;
        inputs.poseRotationDeg[i] =
            robotPose.getRotation().plus(new Rotation2d(noiseTheta)).getDegrees();

        // Simulated ambiguity: 0 for multi-tag (matches PhotonVision multi-tag behavior)
        inputs.poseAmbiguity[i] = targets.size() > 1 ? 0.0 : 0.05 + random.nextDouble() * 0.1;

        // Calculate average distance (using actual robot pose)
        double totalDistance = 0.0;
        for (var target : targets) {
          var tagPose = fieldLayout.getTagPose(target.getFiducialId());
          if (tagPose.isPresent()) {
            totalDistance +=
                robotPose.getTranslation().getDistance(tagPose.get().toPose2d().getTranslation());
          }
        }
        inputs.avgTagDistance[i] = targets.size() > 0 ? totalDistance / targets.size() : 0.0;

        // Score all targets for this camera
        ArrayList<ScoredTarget> cameraScoredTargets = new ArrayList<>();
        for (var target : targets) {
          ScoredTarget scored = scoreTarget(target, i, inputs.poseAmbiguity[i]);
          cameraScoredTargets.add(scored);
        }

        // Find best target for this camera
        if (!cameraScoredTargets.isEmpty()) {
          ScoredTarget cameraBest =
              cameraScoredTargets.stream()
                  .max((a, b) -> Double.compare(a.score, b.score))
                  .orElse(null);

          if (cameraBest != null) {
            inputs.bestTargetId[i] = cameraBest.target.getFiducialId();
            inputs.bestTargetAmbiguity[i] = inputs.poseAmbiguity[i];

            // Store camera-to-target transform
            inputs.bestTargetTransformX[i] = cameraBest.cameraToTarget.getTranslation().getX();
            inputs.bestTargetTransformY[i] = cameraBest.cameraToTarget.getTranslation().getY();
            inputs.bestTargetTransformZ[i] = cameraBest.cameraToTarget.getTranslation().getZ();
            inputs.bestTargetRotationRoll[i] = cameraBest.cameraToTarget.getRotation().getX();
            inputs.bestTargetRotationPitch[i] = cameraBest.cameraToTarget.getRotation().getY();
            inputs.bestTargetRotationYaw[i] = cameraBest.cameraToTarget.getRotation().getZ();

            // Get field pose from layout
            var tagPoseOpt = fieldLayout.getTagPose(cameraBest.target.getFiducialId());
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
        } else {
          inputs.bestTargetId[i] = -1; // No targets
        }
      }
    }

    inputs.visibleTagIds = allVisibleTags.stream().mapToInt(Integer::intValue).toArray();
  }
}
