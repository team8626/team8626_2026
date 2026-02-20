package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;

public class IndexAndShootCommand extends Command {
  private final Shooter shooter;
  private final Indexer indexer;
  private final Drive drive;

  private final double g = 32.174; // ft/s^2
  private final double h = 0;
  private final double flywheelRadiousInInches = 2.0; // TODO: find actual value
  private final double shooterAngle = 68.0; // TODO: TAKE ANGLE INTO ACCOUNT IN
  // CALCULATIONS
  private final Distance hubHeight = Inches.of(80.0);
  private final Translation2d hubPosition =
      new Translation2d(Inches.of(181.56), Inches.of(316.64 / 2));
  private final Transform3d shootertoRobotCenter =
      new Transform3d(
          Inches.of(-8),
          Inches.of(4.5),
          Inches.of(16.25),
          new Rotation3d(new Rotation2d(Degrees.of(0))));

  public IndexAndShootCommand(Shooter shooter, Indexer indexer, Drive drive) {
    this.shooter = shooter;
    this.indexer = indexer;
    this.drive = drive;
    addRequirements(shooter, indexer, drive);
  }

  @Override
  public void initialize() {
    shooter.runVelocity(getShooterVelocityWithAngle(drive.getPose()));
  }

  @Override
  public void execute() {
    // If shooter is fast enough, run indexer to feed balls into shooter
    if (shooter.isAtGoal()) {
      indexer.runVelocity(RPM.of(300)); // TODO: find actual value for indexer velocity
    }
    shooter.runVelocity(getShooterVelocityWithAngle(drive.getPose()));
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    indexer.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // TODO: implement actual logic to determine when finished
  }

  private AngularVelocity getShooterVelocity(
      Pose2d robotPosition) { // TODO: Remove old method if no longer needed
    /** ALL UNITS IN THIS METHOD ARE IN FEET */
    Pose3d shooterPose3d =
        new Pose3d(robotPosition)
            .plus(shootertoRobotCenter); // TODO: verify this transformation is correct
    double Xs =
        Units.metersToFeet(
            shooterPose3d.getTranslation().toTranslation2d().getDistance(hubPosition));
    double Y0 = 0;
    double Ys = Y0 + Units.metersToFeet(shootertoRobotCenter.getZ());
    double Vy = Math.sqrt((h * h) + ((hubHeight.in(Foot) / 12) - Ys) * 32 * 2);
    double tm = (Vy - h) / 32;
    double Vx = Xs / tm;
    double v = Math.sqrt((Vy * Vy) + (Vx * Vx));
    double vRPM = (v * 12) / (Math.PI * flywheelRadiousInInches) * 60;
    return RPM.of(vRPM);
  }

  private AngularVelocity getShooterVelocityWithAngle(Pose2d robotPosition) {
    Pose3d shooterPose3d =
        new Pose3d(robotPosition)
            .plus(shootertoRobotCenter); // TODO: verify this transformation is correct
    double xs = Units.metersToFeet(shooterPose3d.getTranslation().getX());
    double ys = Units.metersToFeet(shooterPose3d.getTranslation().getY());
    double ThetaS = shooterAngle; // TODO: find actual shooter angle
    double xT = 0;
    double yT = hubHeight.in(Foot) / 12; // TODO: Double check the units here w/ Mr. Dumet
    double Dx = xT - xs;
    double Dy = yT - ys;
    double v0 =
        Math.sqrt(
            (g * Math.pow(Dx, 2))
                / ((2 * Math.pow(Math.cos(ThetaS), 2)) * ((Dx) * Math.tan(ThetaS) - (Dy))));
    double vRPM = (v0 * 60 * 12) / (2 * Math.PI * flywheelRadiousInInches);
    return RPM.of(vRPM);
  }
}
