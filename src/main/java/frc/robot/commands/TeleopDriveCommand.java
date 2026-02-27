// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.SlewRateLimiter2d;
import frc.robot.util.TunableControls.TunablePIDController;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.frc2026.FieldConstants;
import org.littletonrobotics.frc2026.util.geometry.AllianceFlipUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Default drive command to run that drives based on controller input */
public class TeleopDriveCommand extends Command {
  private final Drive drive;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier omegaSupplier;
  private final SlewRateLimiter2d driveLimiter;
  private int flipFactor = 1; // 1 for normal, -1 for flipped

  private LinearVelocity maxDriveSpeed = DriveConstants.DEFAULT_DRIVE_SPEED;
  private AngularVelocity maxRotSpeed = DriveConstants.DEFAULT_ROT_SPEED;

  @AutoLogOutput
  private final Trigger inTrenchZoneTrigger = new Trigger(this::inTrenchZone).debounce(0.1);

  @AutoLogOutput
  private final Trigger inBumpZoneTrigger = new Trigger(this::inBumpZone).debounce(0.1);

  @AutoLogOutput private final Trigger hubAimTrigger = RobotContainer.getHubAimTrigger();

  private final TunablePIDController trenchYController =
      new TunablePIDController(DriveConstants.TRENCH_TRANSLATION_CONSTANTS);
  private final TunablePIDController rotationController =
      new TunablePIDController(DriveConstants.ROTATION_CONSTANTS);

  @AutoLogOutput private DriveMode currentDriveMode = DriveMode.NORMAL;

  @AutoLogOutput private double trenchLockAngle = 9999;

  /** Creates a new TeleopDrive. */
  public TeleopDriveCommand(Drive drive, CommandXboxController controller) {
    this.drive = drive;
    this.xSupplier = () -> -controller.getLeftY() * flipFactor;
    this.ySupplier = () -> -controller.getLeftX() * flipFactor;
    this.omegaSupplier = () -> -controller.getRightX();
    this.driveLimiter =
        new SlewRateLimiter2d(DriveConstants.MAX_TELEOP_ACCEL.in(MetersPerSecondPerSecond));

    inTrenchZoneTrigger.onTrue(updateDriveMode(DriveMode.TRENCH_LOCK));
    inBumpZoneTrigger.onTrue(updateDriveMode(DriveMode.BUMP_LOCK));
    // inTrenchZoneTrigger.or(inBumpZoneTrigger).onFalse(updateDriveMode(DriveMode.NORMAL));
    inTrenchZoneTrigger
        .or(inBumpZoneTrigger)
        .onFalse(
            Commands.runOnce(
                () ->
                    currentDriveMode =
                        (hubAimTrigger.getAsBoolean() ? DriveMode.HUB_LOCK : DriveMode.NORMAL)));

    hubAimTrigger
        .whileTrue(updateDriveMode(DriveMode.HUB_LOCK))
        .onFalse(updateDriveMode(DriveMode.NORMAL));

    for (int i = 0; i < 4; i++) {
      Logger.recordOutput("Trench" + i, TeleopDriveConstants.TRENCH_ZONES[i]);
      Logger.recordOutput("Bump" + i, TeleopDriveConstants.BUMP_ZONES[i]);
    }
    addRequirements(drive);
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), ControllerConstants.DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  public boolean inTrenchZone() {
    Pose2d robotPose = drive.getPose();
    for (Translation2d[] zone : TeleopDriveConstants.TRENCH_ZONES) {
      if (robotPose.getX() >= zone[0].getX()
          && robotPose.getX() <= zone[1].getX()
          && robotPose.getY() >= zone[0].getY()
          && robotPose.getY() <= zone[1].getY()) {
        return true;
      }
    }
    return false;
  }

  private Distance getTrenchY() {
    Pose2d robotPose = drive.getPose();
    if (robotPose.getMeasureY().gte(TeleopDriveConstants.FIELD_WIDTH.div(2))) {
      return TeleopDriveConstants.FIELD_WIDTH.minus(TeleopDriveConstants.TRENCH_CENTER);
    }
    return TeleopDriveConstants.TRENCH_CENTER;
  }

  public static Rotation2d getTrenchLockAngle(Rotation2d currentRotation) {
    if (Math.abs(MathUtil.inputModulus(currentRotation.getDegrees(), -180, 180)) > 90) {
      return Rotation2d.fromDegrees(Math.copySign(180, currentRotation.getDegrees()));
    }
    return Rotation2d.kZero;
  }

  public static Rotation2d getBumpLockAngle(Rotation2d currentRotation) {
    for (int i = -135; i < 180; i += 90) {
      if (Math.abs(MathUtil.inputModulus(currentRotation.getDegrees() - i, -180, 180)) <= 45) {
        return Rotation2d.fromDegrees(i);
      }
    }
    return Rotation2d.kZero;
  }

  public boolean inBumpZone() {
    Pose2d robotPose = drive.getPose();
    for (Translation2d[] zone : TeleopDriveConstants.BUMP_ZONES) {
      if (robotPose.getX() >= zone[0].getX()
          && robotPose.getX() <= zone[1].getX()
          && robotPose.getY() >= zone[0].getY()
          && robotPose.getY() <= zone[1].getY()) {
        return true;
      }
    }
    return false;
  }

  private Command updateDriveMode(DriveMode driveMode) {
    return Commands.runOnce(() -> currentDriveMode = driveMode);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flipFactor =
        DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red
            ? -1
            : 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
    linearVelocity = linearVelocity.times(maxDriveSpeed.in(MetersPerSecond));
    linearVelocity = driveLimiter.calculate(linearVelocity);

    switch (currentDriveMode) {
      case NORMAL:
        double omega =
            MathUtil.applyDeadband(omegaSupplier.getAsDouble(), ControllerConstants.DEADBAND);
        omega = Math.copySign(omega * omega, omega); // square for more precise rotation control

        drive.driveFieldCentric(
            MetersPerSecond.of(linearVelocity.getX()),
            MetersPerSecond.of(linearVelocity.getY()),
            maxRotSpeed.times(omega));
        break;
      case TRENCH_LOCK:
        double yVel = trenchYController.calculate(drive.getPose().getY(), getTrenchY().in(Meters));
        double rotSpeedToStraight =
            rotationController.calculate(
                drive.getRotation().getRadians(),
                getTrenchLockAngle(drive.getRotation()).getRadians());
        drive.driveFieldCentric(
            MetersPerSecond.of(linearVelocity.getX()),
            MetersPerSecond.of(yVel),
            RadiansPerSecond.of(rotSpeedToStraight));
        break;
      case BUMP_LOCK:
        double rotSpeedToDiagonal =
            rotationController.calculate(
                drive.getRotation().getRadians(),
                getBumpLockAngle(drive.getRotation()).getRadians());
        drive.driveFieldCentric(
            MetersPerSecond.of(linearVelocity.getX()),
            MetersPerSecond.of(linearVelocity.getY()),
            RadiansPerSecond.of(rotSpeedToDiagonal));
        break;
      case HUB_LOCK:
        double rotSpeedToHub =
            rotationController.calculate(
                drive.getRotation().getRadians(),
                getHubLockAngle(drive.getRotation()).getRadians());
        drive.driveFieldCentric(
            MetersPerSecond.of(linearVelocity.getX()),
            MetersPerSecond.of(linearVelocity.getY()),
            RadiansPerSecond.of(rotSpeedToHub));
        break;
    }
  }

  private void setDriveSpeed(LinearVelocity speed) {
    maxDriveSpeed = speed;
  }

  private void setRotSpeed(AngularVelocity speed) {
    maxRotSpeed = speed;
  }

  public Command speedUpCommand() {
    return Commands.startEnd(
        () -> {
          setDriveSpeed(DriveConstants.FAST_DRIVE_SPEED);
          setRotSpeed(DriveConstants.FAST_ROT_SPEED);
        },
        () -> {
          setDriveSpeed(DriveConstants.DEFAULT_DRIVE_SPEED);
          setRotSpeed(DriveConstants.DEFAULT_ROT_SPEED);
        });
  }

  private Rotation2d getHubLockAngle(Rotation2d currentRotation) {
    Pose2d robotPose = drive.getPose();
    Translation2d hubTranslation =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint)
            .minus(new Translation3d(robotPose.getTranslation()))
            .toTranslation2d();
    Rotation2d angleToHub =
        new Rotation2d(Math.atan2(hubTranslation.getY(), hubTranslation.getX()))
            .plus(Rotation2d.k180deg);
    return angleToHub;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private enum DriveMode {
    NORMAL,
    TRENCH_LOCK,
    BUMP_LOCK,
    HUB_LOCK
  }
}
