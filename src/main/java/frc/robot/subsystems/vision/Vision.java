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

import static frc.robot.subsystems.vision.VisionConstants.maxDistanceMeters;
import static frc.robot.subsystems.vision.VisionConstants.maxPoseAmbiguity;
import static frc.robot.subsystems.vision.VisionConstants.minTagsForMultiTag;
import static frc.robot.subsystems.vision.VisionConstants.thetaStdDevBase;
import static frc.robot.subsystems.vision.VisionConstants.xyStdDevBase;
import static frc.robot.subsystems.vision.VisionConstants.xyStdDevMultiTagDivisor;
import static frc.robot.subsystems.vision.VisionConstants.xyStdDevPerMeter;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.Optional;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

/** Vision subsystem for AprilTag detection and pose estimation. */
public class Vision extends SubsystemBase {
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  // Consumer for sending vision measurements to Drive subsystem
  private final Consumer<VisionMeasurement> visionConsumer;

  // Alerts for camera disconnections
  private final Alert frontCameraDisconnectedAlert =
      new Alert("Front camera disconnected.", AlertType.kWarning);
  private final Alert backCameraDisconnectedAlert =
      new Alert("Back camera disconnected.", AlertType.kWarning);

  /** Container for vision measurement data. */
  public static class VisionMeasurement {
    public final Pose2d pose;
    public final double timestamp;
    public final Matrix<N3, N1> stdDevs;

    public VisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
      this.pose = pose;
      this.timestamp = timestamp;
      this.stdDevs = stdDevs;
    }
  }

  // Supplier for getting current robot pose (for simulation)
  private java.util.function.Supplier<Pose2d> poseSupplier = null;

  /**
   * Create Vision subsystem.
   *
   * @param io Vision IO implementation
   * @param visionConsumer Consumer accepting vision measurements for pose fusion
   */
  public Vision(VisionIO io, Consumer<VisionMeasurement> visionConsumer) {
    this.io = io;
    this.visionConsumer = visionConsumer;
  }

  /**
   * Set pose supplier for simulation. This is used to update the simulation with the current robot
   * pose.
   *
   * @param poseSupplier Supplier providing current robot pose
   */
  public void setPoseSupplier(java.util.function.Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);

    // Update simulation with current robot pose if needed
    if (Constants.currentMode == Constants.Mode.SIM
        && io instanceof VisionIOSim
        && poseSupplier != null) {
      ((VisionIOSim) io).updateRobotPose(poseSupplier.get());
    }

    // Update camera connection alerts
    if (inputs.connected.length >= 2) {
      frontCameraDisconnectedAlert.set(!inputs.connected[0]);
      backCameraDisconnectedAlert.set(!inputs.connected[1]);
    }

    // Process pose estimates from each camera
    for (int i = 0; i < inputs.connected.length; i++) {
      if (!inputs.connected[i] || !inputs.hasTargets[i]) {
        continue;
      }

      // Quality filtering
      if (inputs.poseAmbiguity[i] > maxPoseAmbiguity) {
        Logger.recordOutput("Vision/Camera" + i + "/RejectedAmbiguity", true);
        continue;
      }

      if (inputs.avgTagDistance[i] > maxDistanceMeters) {
        Logger.recordOutput("Vision/Camera" + i + "/RejectedDistance", true);
        continue;
      }

      if (inputs.tagCount[i] < minTagsForMultiTag && inputs.poseAmbiguity[i] > 0.15) {
        Logger.recordOutput("Vision/Camera" + i + "/RejectedSingleTag", true);
        continue;
      }

      // Create pose from inputs
      Pose2d visionPose =
          new Pose2d(
              inputs.poseX[i], inputs.poseY[i], Rotation2d.fromDegrees(inputs.poseRotationDeg[i]));

      // Calculate standard deviations based on distance and tag count
      Matrix<N3, N1> stdDevs = calculateStdDevs(inputs.avgTagDistance[i], inputs.tagCount[i]);

      // Send measurement to Drive subsystem
      visionConsumer.accept(new VisionMeasurement(visionPose, inputs.timestampSeconds[i], stdDevs));

      // Log accepted measurement
      Logger.recordOutput("Vision/Camera" + i + "/AcceptedPose", visionPose);
      Logger.recordOutput(
          "Vision/Camera" + i + "/StdDevs",
          new double[] {stdDevs.get(0, 0), stdDevs.get(1, 0), stdDevs.get(2, 0)});

      // Log best target for AdvantageScope visualization
      if (inputs.bestTargetId[i] >= 0) {
        Pose3d bestTargetFieldPose =
            new Pose3d(
                inputs.bestTargetFieldX[i],
                inputs.bestTargetFieldY[i],
                inputs.bestTargetFieldZ[i],
                new Rotation3d(
                    inputs.bestTargetFieldRotRoll[i],
                    inputs.bestTargetFieldRotPitch[i],
                    inputs.bestTargetFieldRotYaw[i]));

        Logger.recordOutput("Vision/Camera" + i + "/BestTarget/FieldPose", bestTargetFieldPose);
        Logger.recordOutput("Vision/Camera" + i + "/BestTarget/TagID", inputs.bestTargetId[i]);
        Logger.recordOutput(
            "Vision/Camera" + i + "/BestTarget/Ambiguity", inputs.bestTargetAmbiguity[i]);

        // Distance for debugging
        double distance =
            Math.sqrt(
                inputs.bestTargetTransformX[i] * inputs.bestTargetTransformX[i]
                    + inputs.bestTargetTransformY[i] * inputs.bestTargetTransformY[i]
                    + inputs.bestTargetTransformZ[i] * inputs.bestTargetTransformZ[i]);
        Logger.recordOutput("Vision/Camera" + i + "/BestTarget/Distance", distance);
      }
    }
  }

  /**
   * Calculate standard deviations for vision measurement.
   *
   * <p>Standard deviation increases with distance and decreases with more tags.
   *
   * @param avgDistance Average distance to visible tags in meters
   * @param tagCount Number of tags in view
   * @return Standard deviation matrix [x, y, theta]
   */
  private Matrix<N3, N1> calculateStdDevs(double avgDistance, int tagCount) {
    // Base std dev increases with distance
    double xyStdDev = xyStdDevBase + (avgDistance * xyStdDevPerMeter);
    double thetaStdDev = thetaStdDevBase + (avgDistance * xyStdDevPerMeter);

    // Multi-tag is more accurate - divide by factor
    if (tagCount >= minTagsForMultiTag) {
      xyStdDev /= xyStdDevMultiTagDivisor;
      thetaStdDev /= xyStdDevMultiTagDivisor;
    }

    return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
  }

  /**
   * Get best target's field-relative Pose3d.
   *
   * @param cameraIndex Camera index (0=front, 1=back)
   * @return Optional with Pose3d if valid target exists
   */
  public Optional<Pose3d> getBestTargetFieldPose(int cameraIndex) {
    if (cameraIndex < 0
        || cameraIndex >= inputs.bestTargetId.length
        || inputs.bestTargetId[cameraIndex] < 0) {
      return Optional.empty();
    }

    return Optional.of(
        new Pose3d(
            inputs.bestTargetFieldX[cameraIndex],
            inputs.bestTargetFieldY[cameraIndex],
            inputs.bestTargetFieldZ[cameraIndex],
            new Rotation3d(
                inputs.bestTargetFieldRotRoll[cameraIndex],
                inputs.bestTargetFieldRotPitch[cameraIndex],
                inputs.bestTargetFieldRotYaw[cameraIndex])));
  }

  /**
   * Check if camera has valid best target.
   *
   * @param cameraIndex Camera index (0=front, 1=back)
   * @return true if camera has valid best target
   */
  public boolean hasBestTarget(int cameraIndex) {
    return cameraIndex >= 0
        && cameraIndex < inputs.bestTargetId.length
        && inputs.bestTargetId[cameraIndex] >= 0;
  }
}
