package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.shooter.ShooterConstants;
import org.littletonrobotics.frc2026.FieldConstants;
import org.littletonrobotics.frc2026.util.geometry.AllianceFlipUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterCommandsUtil {
  private static final double g = 32.174; // ft/s^2

  /**
   * Calculates the required shooter velocity to hit the center of the hub from the given robot
   * position.
   *
   * @param robotPosition The current position of the robot on the field.
   * @return The required shooter velocity.
   */
  public static AngularVelocity getShooterVelocitytoHub(Pose2d robotPosition) {

    return getShooterVelocityToTarget(robotPosition, FieldConstants.Hub.topCenterPoint);
  }

  /**
   * Calculates the required shooter velocity to hit the specified target from the given robot
   * position.
   *
   * @param robotPose The current position of the robot on the field.
   * @param new_targetPose The position of the target in field coordinates.
   * @return The required shooter velocity.
   */
  @AutoLogOutput
  public static AngularVelocity getShooterVelocityToTarget(
      Pose2d robotPose, Translation3d new_targetPose) {

    Translation3d targetPose = (AllianceFlipUtil.apply(new_targetPose));

    Pose3d shooterPose3d = new Pose3d(robotPose).plus(ShooterConstants.shootertoRobotCenter);

    // Robot position in shooting plane coordinates (origin at target, x forward, y up)
    double x0 =
        -Units.metersToFeet(
            new Translation3d(robotPose.getX(), robotPose.getY(), 0)
                .toTranslation2d()
                .getDistance(targetPose.toTranslation2d()));
    // Shooter position in shooting plane coordinates (origin at target, x forward, y up)
    double xs =
        -Units.metersToFeet(
            shooterPose3d
                .getTranslation()
                .toTranslation2d()
                .getDistance(targetPose.toTranslation2d()));

    double ys = Units.metersToFeet(shooterPose3d.getTranslation().getZ());

    // Shooter angle and target position
    double ThetaS = ShooterConstants.shooterAngle.in(Radians);

    // Target position in shooting plane coordinates (origin at target, x forward, y up)
    double xT = 0;
    double yT = Units.metersToFeet(targetPose.getZ());

    // Calculate distance to target in shooting plane coordinates
    double Dx = xT - xs;
    double Dy = yT - ys;

    // double denom = ((xT - xs) * Math.tan(ThetaS) - (yT - ys));
    // if (denom <= 0)
    //   throw new IllegalArgumentException(
    //       "Invalid shot geometry: check shooter angle and target position");
    double v0 =
        Math.sqrt(
            (g * Math.pow(Dx, 2))
                / ((2 * Math.pow(Math.cos(ThetaS), 2)) * (Dx * Math.tan(ThetaS) - Dy)));

    double vRPM = (v0 * 60.0) / (2.0 * Math.PI * ShooterConstants.flywheelRadius.in(Feet));

    Logger.recordOutput("getShooterVelocityToTarget/Pose Robot", robotPose);
    Logger.recordOutput("getShooterVelocityToTarget/Pose Shooter", shooterPose3d);
    Logger.recordOutput("getShooterVelocityToTarget/Pose Target", targetPose);
    Logger.recordOutput("getShooterVelocityToTarget/Robot Distance x0", x0, "feet");
    Logger.recordOutput("getShooterVelocityToTarget/Shooter Distance Dx", Dx, "feet");
    Logger.recordOutput("getShooterVelocityToTarget/Shooter Distance Dy", Dy, "feet");
    Logger.recordOutput("getShooterVelocityToTarget/Velocity Launch", v0, "feet per second");
    Logger.recordOutput("getShooterVelocityToTarget/Velocity Flywheel", vRPM, "RPM");

    return RPM.of(vRPM);
  }
}
