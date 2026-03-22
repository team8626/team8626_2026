// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.subsystems.drive.AkitDrive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveConstants.DriveSpeed;
import frc.robot.util.SlewRateLimiter2d;
import frc.robot.util.TunableControls.TunablePIDController;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Default drive command to run that drives based on controller input */
public class TeleopDriveCommand extends Command {
  private final AkitDrive drive;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier omegaSupplier;
  private final SlewRateLimiter2d driveLimiter;
  private int flipFactor = 1; // 1 for normal, -1 for flipped

  private LinearVelocity maxDriveSpeed = DriveConstants.DEFAULT_DRIVE_SPEED;
  private AngularVelocity maxRotSpeed = DriveConstants.DEFAULT_ROT_SPEED;
  private double rotSpeedToHub = 0.0;

  @AutoLogOutput private DriveSpeed driveSpeed = DriveSpeed.DEFAULT;

  @AutoLogOutput
  private final Trigger inTrenchZoneTrigger = new Trigger(this::inTrenchZone).debounce(0.1);

  @AutoLogOutput
  private final Trigger inBumpZoneTrigger = new Trigger(this::inBumpZone).debounce(0.1);

  @AutoLogOutput private BooleanSupplier forceTargetAim = () -> false;
  @AutoLogOutput private BooleanSupplier forceHubLockThenX = () -> false;

  @AutoLogOutput
  private final Trigger hubAimTrigger =
      RobotContainer.getHubAimTrigger().or(() -> forceTargetAim.getAsBoolean());

  @AutoLogOutput
  private final Trigger hubAimInPlaceTrigger = RobotContainer.getHubAimInPlaceTrigger();

  private final TunablePIDController trenchYController =
      new TunablePIDController(DriveConstants.TRENCH_TRANSLATION_CONSTANTS);
  private final TunablePIDController rotationController =
      new TunablePIDController(DriveConstants.ROTATION_CONSTANTS);

  @AutoLogOutput private DriveMode currentDriveMode = DriveMode.NORMAL;

  @AutoLogOutput private double trenchLockAngle = 9999;

  /** Creates a new TeleopDrive. */
  public TeleopDriveCommand(AkitDrive drive, CommandXboxController controller) {
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

    if (hubAimTrigger != null) {
      hubAimTrigger
          .whileTrue(updateDriveMode(DriveMode.HUB_LOCK))
          .onFalse(updateDriveMode(DriveMode.NORMAL));
    }

    if (hubAimInPlaceTrigger != null) {
      hubAimInPlaceTrigger
          .whileTrue(updateDriveMode(DriveMode.HUB_LOCK_IN_PLACE))
          .onFalse(updateDriveMode(DriveMode.NORMAL));
    }

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

    // Forcing the drice to X
    boolean shouldStopWithX =
        forceHubLockThenX.getAsBoolean() && ShooterCommandsUtil.inPositionToShoot(drive);
    if (shouldStopWithX) {
      drive.stopWithX();
      return;
    }

    Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
    linearVelocity = linearVelocity.times(maxDriveSpeed.in(MetersPerSecond));
    linearVelocity = driveLimiter.calculate(linearVelocity);

    switch (currentDriveMode) {
      case NORMAL, TRENCH_LOCK, BUMP_LOCK:
        double omega =
            MathUtil.applyDeadband(omegaSupplier.getAsDouble(), ControllerConstants.DEADBAND);
        omega = Math.copySign(omega * omega, omega); // square for more precise rotation control

        drive.driveFieldCentric(
            MetersPerSecond.of(linearVelocity.getX()),
            MetersPerSecond.of(linearVelocity.getY()),
            maxRotSpeed.times(omega));
        break;
      case HUB_LOCK:
        rotSpeedToHub =
            rotationController.calculate(
                drive.getRotation().getRadians(),
                ShooterCommandsUtil.getHubLockAngle(drive).getRadians());
        drive.driveFieldCentric(
            MetersPerSecond.of(linearVelocity.getX()),
            MetersPerSecond.of(linearVelocity.getY()),
            RadiansPerSecond.of(rotSpeedToHub));
        break;
      case HUB_LOCK_IN_PLACE:
        rotSpeedToHub =
            rotationController.calculate(
                drive.getRotation().getRadians(),
                ShooterCommandsUtil.getHubLockAngle(drive).getRadians());
        drive.driveFieldCentric(
            MetersPerSecond.of(0), MetersPerSecond.of(0), RadiansPerSecond.of(rotSpeedToHub));
        break;
    }
  }

  private void setDriveSpeed(LinearVelocity speed) {
    maxDriveSpeed = speed;
  }

  private void setRotSpeed(AngularVelocity speed) {
    maxRotSpeed = speed;
  }

  private void setSpeed(DriveSpeed new_driveSpeed) {
    driveSpeed = new_driveSpeed;
    switch (new_driveSpeed) {
      case SLOW -> {
        maxDriveSpeed = DriveConstants.SLOW_DRIVE_SPEED;
        maxRotSpeed = DriveConstants.SLOW_ROT_SPEED;
      }
      case FAST -> {
        maxDriveSpeed = DriveConstants.FAST_DRIVE_SPEED;
        maxRotSpeed = DriveConstants.FAST_ROT_SPEED;
      }
      case INTAKE -> {
        maxDriveSpeed = DriveConstants.INTAKE_DRIVE_SPEED;
        maxRotSpeed = DriveConstants.INTAKE_ROT_SPEED;
      }
      default -> {
        maxDriveSpeed = DriveConstants.DEFAULT_DRIVE_SPEED;
        maxRotSpeed = DriveConstants.DEFAULT_ROT_SPEED;
      }
    }
  }

  // @Deprecated
  // private Rotation2d getHubLockAngle(AkitDrive drive) {
  //   Pose2d robotPose = drive.getPose();
  //   Translation2d hubTranslation =
  //       AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint)
  //           .minus(new Translation3d(robotPose.getTranslation()))
  //           .toTranslation2d();
  //   Rotation2d angleToHub =
  //       new Rotation2d(Math.atan2(hubTranslation.getY(), hubTranslation.getX()))
  //           .plus(Rotation2d.kZero);
  //   return angleToHub;
  // }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // TODO: Set top X and reset drive mode here?
    // if(currentDriveMode == DriveMode.HUB_LOCK_IN_PLACE) {
    //   drive.stopWithX();
    //   currentDriveMode = DriveMode.NORMAL;
    // }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  // private boolean inPositionToShoot() {
  //   return Math.abs(
  //           MathUtil.inputModulus(
  //               drive.getRotation().getDegrees() - getHubLockAngle(drive).getDegrees(), -180,
  // 180))
  //       < (DriveConstants.AIM_TOLERANCE_DEG.get() / 2);
  // }

  public void setForceTargetAim(boolean enabled) {
    this.forceTargetAim = () -> enabled;
  }

  public void setForceHubLockThenX(boolean enabled) {
    this.forceHubLockThenX = () -> enabled;
  }

  public Command withSpeed(DriveSpeed speed) {
    return Commands.startEnd(() -> setSpeed(speed), () -> setSpeed(DriveSpeed.DEFAULT));
  }

  public Command withHubLock() {
    return Commands.startEnd(
        () -> {
          setForceTargetAim(true);
          currentDriveMode = DriveMode.HUB_LOCK;
        },
        () -> {
          setForceTargetAim(false);
          currentDriveMode = DriveMode.NORMAL;
        });
  }

  public Command withHubLockThenX() {
    return Commands.startEnd(
        () -> {
          setForceTargetAim(true);
          setForceHubLockThenX(true);
          currentDriveMode = DriveMode.HUB_LOCK;
        },
        () -> {
          setForceTargetAim(false);
          setForceHubLockThenX(false);
          currentDriveMode = DriveMode.NORMAL;
        });
  }

  public Command withHubLock(Command command) {
    return command.alongWith(withHubLock());
  }

  public Command withHubLockThenX(Command command) {
    return command.alongWith(withHubLockThenX());
  }

  public Command withSpeed(DriveSpeed speed, Command command) {
    return command.alongWith(withSpeed(speed));
  }

  private enum DriveMode {
    NORMAL,
    TRENCH_LOCK,
    BUMP_LOCK,
    HUB_LOCK,
    HUB_LOCK_IN_PLACE
  }
}
