package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.ShooterTargetConstants;
import frc.robot.subsystems.anotherShooter.AnotherShooterConstants;
import frc.robot.subsystems.drive.AkitDrive;
import frc.robot.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2026.FieldConstants;
import org.littletonrobotics.frc2026.util.geometry.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

public class ShooterCommandsUtil {
  public static record ShooterData(
      AngularVelocity velocityShooter, AngularVelocity velocityIndexer) {}

  /**
   * Helper methods to calculate the distance to the hub or target. This can be used in commands to
   * determine the appropriate shooter and indexer velocities.
   */
  public static Distance getDistToHub(AkitDrive drive) {
    return getDistToTarget(drive, FieldConstants.Hub.topCenterPoint);
  }

  public static Distance getDistToTarget(AkitDrive drive, Translation3d target) {
    double distToTargetMeters =
        drive
            .getPose()
            .getTranslation()
            .getDistance(AllianceFlipUtil.apply(target.toTranslation2d()));

    return Meters.of(distToTargetMeters);
  }

  /**
   * Helper methods for calculating shooter velocities using the treemap. These methods can be used
   * in commands to calculate the required shooter and indexer velocities based on the robot's
   * current position and the target position.
   */
  public static ShooterData calculateRPMToHub(AkitDrive drive) {
    return calculateRPMToTarget(drive, FieldConstants.Hub.topCenterPoint);
  }

  public static ShooterData calculateRPMToPassingDepotSide(AkitDrive drive) {
    return calculateRPMToTarget(drive, ShooterTargetConstants.TARGET_PASSING_DEPOT_SIDE);
  }

  public static ShooterData calculateRPMToPassingOutpostSide(AkitDrive drive) {
    return calculateRPMToTarget(drive, ShooterTargetConstants.TARGET_PASSING_OUTPOST_SIDE);
  }

  public static Translation3d getPassingTarget(AkitDrive drive) {
    Translation3d depotTarget = ShooterTargetConstants.TARGET_PASSING_DEPOT_SIDE;
    Translation3d outpostTarget = ShooterTargetConstants.TARGET_PASSING_OUTPOST_SIDE;

    double depotDist = getDistToTarget(drive, depotTarget).in(Meters);
    double outpostDist = getDistToTarget(drive, outpostTarget).in(Meters);

    Translation3d selected = depotDist <= outpostDist ? depotTarget : outpostTarget;

    Logger.recordOutput("AnotherShooter/Passing/SelectedTarget", AllianceFlipUtil.apply(selected));
    Logger.recordOutput("AnotherShooter/Passing/DepotDistanceM", depotDist);
    Logger.recordOutput("AnotherShooter/Passing/OutpostDistanceM", outpostDist);

    return selected;
  }

  public static ShooterData calculateRPMToPassing(AkitDrive drive) {
    return calculateRPMToTarget(drive, getPassingTarget(drive));
  }

  public static ShooterData calculateRPMToTarget(AkitDrive drive, Translation3d target) {
    double distToTargetFeet = getDistToTarget(drive, target).in(Feet);

    AngularVelocity velocityShooter = RPM.of(AnotherShooterConstants.RPMMap.get(distToTargetFeet));
    AngularVelocity velocityIndexer =
        RPM.of(AnotherShooterConstants.IndexerMap.get(distToTargetFeet));

    Logger.recordOutput("AnotherShooter/Target/Pose Robot", drive.getPose());
    Logger.recordOutput("AnotherShooter/Target/Pose Target", AllianceFlipUtil.apply(target));
    Logger.recordOutput("AnotherShooter/Target/Distance to Target", distToTargetFeet, "feet");
    Logger.recordOutput("AnotherShooter/Target/Shooter RPM", velocityShooter.in(RPM), "RPM");
    Logger.recordOutput(
        "AnotherShooter/Target/Indexer RPS",
        velocityIndexer.in(RotationsPerSecond),
        "rotations per second");

    return new ShooterData(velocityShooter, velocityIndexer);
  }
  /**
   * Helper method to calculate the angle to lock onto the hub or target. This can be used in
   * commands to rotate the robot to face the hub or target for shooting.
   */
  public static Rotation2d getHubLockAngle(AkitDrive drive) {
    return getTargetLockAngle(drive, FieldConstants.Hub.topCenterPoint);
  }

  public static Rotation2d getPassingLockAngle(AkitDrive drive) {
    return getTargetLockAngle(drive, getPassingTarget(drive));
  }

  public static Rotation2d getTargetLockAngle(AkitDrive drive, Translation3d target) {
    Pose2d robotPose = drive.getPose();
    Pose3d shooterPose3d =
        new Pose3d(robotPose).plus(AnotherShooterConstants.ANOTHERSHOOTER_OFFSET);

    Translation2d shooterTranslation = shooterPose3d.getTranslation().toTranslation2d();
    Translation2d targetTranslation = AllianceFlipUtil.apply(target).toTranslation2d();

    Translation2d delta = targetTranslation.minus(shooterTranslation);
    Rotation2d targetLockAngle = new Rotation2d(Math.atan2(delta.getY(), delta.getX()));

    Logger.recordOutput("AnotherShooter/Target/Pose Shooter", shooterPose3d);
    Logger.recordOutput(
        "AnotherShooter/Target/Pose Target (Angle Lock)", AllianceFlipUtil.apply(target));
    Logger.recordOutput(
        "AnotherShooter/Target/Lock Angle", targetLockAngle.getDegrees(), "degrees");

    return targetLockAngle;
  }
  /** Helper method to determine if the robot is in position to shoot. */
  public static boolean inPositionToShoot(AkitDrive drive, Translation3d target) {
    double angleErrorDeg =
        MathUtil.inputModulus(
            drive.getRotation().getDegrees() - getTargetLockAngle(drive, target).getDegrees(),
            -180,
            180);

    double omegaDegPerSec = Math.toDegrees(drive.getChassisSpeeds().omegaRadiansPerSecond);

    Logger.recordOutput("AnotherShooter/Target/DegToTarget", angleErrorDeg);
    Logger.recordOutput("AnotherShooter/Target/OmegaDegPerSec", omegaDegPerSec);

    return Math.abs(angleErrorDeg) <= DriveConstants.AIM_TOLERANCE_DEG.get()
        && Math.abs(omegaDegPerSec) <= DriveConstants.AIM_MAX_ANGULAR_VEL_DEG_PER_SEC.get();
  }

  public static boolean inPositionToShoot(AkitDrive drive) {
    return inPositionToShoot(drive, FieldConstants.Hub.topCenterPoint);
  }

  public static boolean isInAllianceZone(AkitDrive drive) {
    var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return false;
    }

    double robotX = drive.getPose().getX();

    return switch (alliance.get()) {
      case Blue -> robotX <= FieldConstants.LinesVertical.allianceZone;
      case Red -> robotX >= FieldConstants.LinesVertical.oppAllianceZone;
    };
  }

  private static boolean isAllowedToShoot(AkitDrive drive, Translation3d target) {
    return isInAllianceZone(drive) && inPositionToShoot(drive, target);
  }

  private static boolean isAllowedToShoot(AkitDrive drive) {
    return isAllowedToShoot(drive, FieldConstants.Hub.topCenterPoint);
  }
}
