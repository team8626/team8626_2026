// Copyright (c) 2026 team8626
// https://github.com/team8626
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import static frc.robot.subsystems.vision.VisionConstants.alignAngularKd;
import static frc.robot.subsystems.vision.VisionConstants.alignAngularKp;
import static frc.robot.subsystems.vision.VisionConstants.alignAngularMaxAcceleration;
import static frc.robot.subsystems.vision.VisionConstants.alignAngularMaxVelocity;
import static frc.robot.subsystems.vision.VisionConstants.alignLinearKp;
import static frc.robot.subsystems.vision.VisionConstants.alignMaxLinearSpeedMetersPerSec;
import static frc.robot.subsystems.vision.VisionConstants.alignToTagDistanceMeters;
import static frc.robot.subsystems.vision.VisionConstants.alignToleranceMeters;
import static frc.robot.subsystems.vision.VisionConstants.alignToleranceRadians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import java.util.Optional;

/**
 * Command that aligns the robot to a given camera's best AprilTag target: robot moves to a pose in
 * front of the tag at a configurable distance. For the front camera, the robot's front faces the
 * tag; for the rear camera, the robot's rear faces the tag (so the rear aligns to the tag). Usable
 * in both autonomous and teleop (e.g. while-held).
 *
 * <p>When the command starts, that camera's best target is captured and the robot aligns to it
 * until the command ends. If the camera has no best target at start, the command does nothing and
 * ends immediately.
 */
public class AlignToTargetCommand extends Command {
  /** Camera index for front camera (0). */
  public static final int FRONT_CAMERA = 0;
  /** Camera index for back camera (1). */
  public static final int BACK_CAMERA = 1;

  private final Drive drive;
  private final Vision vision;
  private final int cameraIndex;
  private final double distanceMeters;
  private final ProfiledPIDController angleController;

  /** Desired pose captured at start; null if the camera had no best target when command started. */
  private Pose2d desiredPoseAtStart = null;

  private AlignToTargetCommand(Drive drive, Vision vision, int cameraIndex, double distanceMeters) {
    this.drive = drive;
    this.vision = vision;
    this.cameraIndex = cameraIndex;
    this.distanceMeters = distanceMeters;
    this.angleController =
        new ProfiledPIDController(
            alignAngularKp,
            0.0,
            alignAngularKd,
            new TrapezoidProfile.Constraints(alignAngularMaxVelocity, alignAngularMaxAcceleration));
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drive, vision);
  }

  /**
   * Align to the front camera's best target using the default distance from VisionConstants.
   *
   * @param drive Drive subsystem
   * @param vision Vision subsystem
   * @return command; does nothing and ends immediately if the front camera has no best target
   */
  public static Command alignToFrontCamera(Drive drive, Vision vision) {
    return alignToCameraBestTarget(drive, vision, FRONT_CAMERA);
  }

  /**
   * Align to the back camera's best target using the default distance from VisionConstants.
   *
   * @param drive Drive subsystem
   * @param vision Vision subsystem
   * @return command; does nothing and ends immediately if the back camera has no best target
   */
  public static Command alignToBackCamera(Drive drive, Vision vision) {
    return alignToCameraBestTarget(drive, vision, BACK_CAMERA);
  }

  /**
   * Align to a given camera's best target.
   *
   * @param drive Drive subsystem
   * @param vision Vision subsystem
   * @param cameraIndex Camera index (0 = front, 1 = back)
   * @return command; does nothing and ends immediately if that camera has no best target
   */
  public static Command alignToCameraBestTarget(Drive drive, Vision vision, int cameraIndex) {
    return alignToCameraBestTarget(drive, vision, cameraIndex, alignToTagDistanceMeters);
  }

  /**
   * Align to a given camera's best target at the specified distance from the tag.
   *
   * @param drive Drive subsystem
   * @param vision Vision subsystem
   * @param cameraIndex Camera index (0 = front, 1 = back)
   * @param distanceMeters Desired distance in meters from robot center to tag
   * @return command; does nothing and ends immediately if that camera has no best target
   */
  public static Command alignToCameraBestTarget(
      Drive drive, Vision vision, int cameraIndex, double distanceMeters) {
    return new AlignToTargetCommand(drive, vision, cameraIndex, distanceMeters);
  }

  /**
   * Returns the desired distance in meters. Uses Preferences when available so operators can tune
   * without recompile; otherwise uses the constant passed at construction.
   */
  private double getEffectiveDistanceMeters() {
    return Preferences.getDouble("AlignToTagDistanceMeters", distanceMeters);
  }

  /**
   * Compute desired robot Pose2d from tag field pose: robot in front of tag. Position is always "in
   * front of" the tag (distance meters along the direction the tag faces). Orientation depends on
   * camera: front camera = robot's front faces the tag (heading tagYaw + PI); rear camera = robot's
   * rear faces the tag (heading tagYaw, so front faces away from tag).
   *
   * @param tagPose Tag field pose
   * @param distanceMeters Distance from robot to tag
   * @param frontFacesTag true for front camera (front faces tag), false for rear camera (rear faces
   *     tag)
   */
  private static Pose2d desiredPoseFromTag(
      Pose3d tagPose, double distanceMeters, boolean frontFacesTag) {
    double tagX = tagPose.getX();
    double tagY = tagPose.getY();
    double tagYaw = tagPose.getRotation().getZ();
    double dx = Math.cos(tagYaw) * distanceMeters;
    double dy = Math.sin(tagYaw) * distanceMeters;
    // Robot position: in front of tag (along tag's forward direction)
    double robotX = tagX + dx;
    double robotY = tagY + dy;
    // Front camera: front faces tag (heading = tagYaw + PI). Rear camera: rear faces tag (heading =
    // tagYaw).
    double robotYaw =
        frontFacesTag ? MathUtil.angleModulus(tagYaw + Math.PI) : MathUtil.angleModulus(tagYaw);
    return new Pose2d(robotX, robotY, Rotation2d.fromRadians(robotYaw));
  }

  @Override
  public void initialize() {
    angleController.reset(drive.getRotation().getRadians());
    desiredPoseAtStart = null;
    if (vision.hasBestTarget(cameraIndex)) {
      Optional<Pose3d> tagOpt = vision.getBestTargetFieldPose(cameraIndex);
      if (tagOpt.isPresent()) {
        boolean frontFacesTag = (cameraIndex == FRONT_CAMERA);
        desiredPoseAtStart =
            desiredPoseFromTag(tagOpt.get(), getEffectiveDistanceMeters(), frontFacesTag);
      }
    }
  }

  @Override
  public void execute() {
    if (desiredPoseAtStart == null) {
      drive.runVelocity(new ChassisSpeeds());
      return;
    }

    Pose2d desired = desiredPoseAtStart;
    Pose2d current = drive.getPose();

    double xError = desired.getX() - current.getX();
    double yError = desired.getY() - current.getY();
    double vx = alignLinearKp * xError;
    double vy = alignLinearKp * yError;

    double magnitude = Math.hypot(vx, vy);
    if (magnitude > alignMaxLinearSpeedMetersPerSec) {
      double scale = alignMaxLinearSpeedMetersPerSec / magnitude;
      vx *= scale;
      vy *= scale;
    }

    double omega =
        angleController.calculate(
            current.getRotation().getRadians(), desired.getRotation().getRadians());

    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, drive.getRotation());
    drive.runVelocity(speeds);
  }

  @Override
  public boolean isFinished() {
    if (desiredPoseAtStart == null) {
      return true; // No valid target at start, end immediately
    }

    Pose2d desired = desiredPoseAtStart;
    Pose2d current = drive.getPose();

    double xError = desired.getX() - current.getX();
    double yError = desired.getY() - current.getY();
    double posError = Math.hypot(xError, yError);
    double angleError =
        MathUtil.inputModulus(
            desired.getRotation().getRadians() - current.getRotation().getRadians(),
            -Math.PI,
            Math.PI);

    return posError < alignToleranceMeters && Math.abs(angleError) < alignToleranceRadians;
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds());
  }
}
