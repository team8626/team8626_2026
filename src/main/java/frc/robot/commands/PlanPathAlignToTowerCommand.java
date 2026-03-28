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

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberConstants.ClimbPosition;
import frc.robot.subsystems.drive.AkitDrive;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.frc2026.util.geometry.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

/**
 * Drives the robot to the nearest tower climbing position using PathPlanner pathfinding. Enables
 * vision climbing mode to bias AprilTag trust toward the tower tags (blue: 31, 32 / red: 15, 16)
 * for more accurate pose estimation during the approach.
 */
public class PlanPathAlignToTowerCommand extends Command {

  // Conservative constraints for precise tower alignment
  private static final PathConstraints ALIGN_CONSTRAINTS =
      new PathConstraints(2.0, 1.5, Math.toRadians(540), Math.toRadians(720));

  private final AkitDrive akitDrive;
  private final Vision vision;

  private Command pathfindCommand = null;
  private Pose2d targetPose = null;

  /**
   * Creates a new AlignToTowerCommand.
   *
   * @param akitDrive The swerve drive subsystem.
   * @param vision The vision subsystem (used to enable climbing mode for tag biasing).
   */
  public PlanPathAlignToTowerCommand(AkitDrive akitDrive, Vision vision) {
    this.akitDrive = akitDrive;
    this.vision = vision;

    addRequirements(akitDrive);
  }

  @Override
  public void initialize() {
    // Enable climbing vision mode to trust tower AprilTags more
    vision.setClimbing(true);

    // Find the nearest climb position (alliance-flipped)
    Pose2d robotPose = akitDrive.getPose();
    double minDistance = Double.MAX_VALUE;
    targetPose = null;

    for (ClimbPosition position : ClimbPosition.values()) {
      Pose2d flippedPose = AllianceFlipUtil.apply(position.getPose());
      double distance = robotPose.getTranslation().getDistance(flippedPose.getTranslation());

      if (distance < minDistance) {
        minDistance = distance;
        targetPose = flippedPose;
      }
    }

    // Log the selected target
    Logger.recordOutput("PlanPathAlignToTower/TargetPose", targetPose);
    Logger.recordOutput("PlanPathAlignToTower/DistanceToTarget", minDistance);

    // Create and start the pathfinding command
    pathfindCommand = AutoBuilder.pathfindToPose(targetPose, ALIGN_CONSTRAINTS);
    pathfindCommand.initialize();
  }

  @Override
  public void execute() {
    if (pathfindCommand != null) {
      pathfindCommand.execute();
    }

    // Log current distance to target for monitoring
    if (targetPose != null) {
      double currentDistance =
          akitDrive.getPose().getTranslation().getDistance(targetPose.getTranslation());
      Logger.recordOutput("PlanPathAlignToTower/CurrentDistance", currentDistance);
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (pathfindCommand != null) {
      pathfindCommand.end(interrupted);
    }

    // Restore normal vision mode
    vision.setClimbing(false);

    Logger.recordOutput("PlanPathAlignToTower/Interrupted", interrupted);
  }

  @Override
  public boolean isFinished() {
    return pathfindCommand != null && pathfindCommand.isFinished();
  }
}
