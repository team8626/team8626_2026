package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

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
import frc.robot.subsystems.drive.AkitDrive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveConstants.DriveSpeed;
import frc.robot.util.SlewRateLimiter2d;
import frc.robot.util.TunableControls.TunablePIDController;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.frc2026.FieldConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Default drive command to run that drives based on controller input */
public class TeleopDriveCommand extends Command {
  private static final Supplier<Translation3d> HUB_TARGET_SUPPLIER =
      () -> FieldConstants.Hub.topCenterPoint;

  private final AkitDrive drive;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier omegaSupplier;
  private final SlewRateLimiter2d driveLimiter;
  private int flipFactor = 1; // 1 for normal, -1 for flipped

  private LinearVelocity maxDriveSpeed = DriveConstants.DEFAULT_DRIVE_SPEED;
  private AngularVelocity maxRotSpeed = DriveConstants.DEFAULT_ROT_SPEED;
  private double rotSpeedToTarget = 0.0;

  @AutoLogOutput private DriveSpeed driveSpeed = DriveSpeed.DEFAULT;

  @AutoLogOutput
  private final Trigger inTrenchZoneTrigger = new Trigger(this::inTrenchZone).debounce(0.1);

  @AutoLogOutput
  private final Trigger inBumpZoneTrigger = new Trigger(this::inBumpZone).debounce(0.1);

  @AutoLogOutput private boolean forceTargetTracking = false;
  @AutoLogOutput private boolean forceTargetLockThenX = false;

  @AutoLogOutput private final Trigger hubTrackTrigger = new Trigger(() -> forceTargetTracking);
  @AutoLogOutput private final Trigger hubAimTrigger = RobotContainer.getAimTrigger();

  private Supplier<Translation3d> targetSupplier = HUB_TARGET_SUPPLIER;

  private final TunablePIDController trenchYController =
      new TunablePIDController(DriveConstants.TRENCH_TRANSLATION_CONSTANTS);
  private final TunablePIDController rotationController =
      new TunablePIDController(DriveConstants.ROTATION_CONSTANTS);

  @AutoLogOutput private DriveMode currentDriveMode = DriveMode.NORMAL;
  @AutoLogOutput private double trenchLockAngle = 9999;

  public TeleopDriveCommand(AkitDrive drive, CommandXboxController controller) {
    this.drive = drive;
    this.xSupplier = () -> -controller.getLeftY() * flipFactor;
    this.ySupplier = () -> -controller.getLeftX() * flipFactor;
    this.omegaSupplier = () -> -controller.getRightX();
    this.driveLimiter =
        new SlewRateLimiter2d(DriveConstants.MAX_TELEOP_ACCEL.in(MetersPerSecondPerSecond));

    inTrenchZoneTrigger.onTrue(updateDriveMode(DriveMode.TRENCH_LOCK));
    inBumpZoneTrigger.onTrue(updateDriveMode(DriveMode.BUMP_LOCK));
    inTrenchZoneTrigger
        .or(inBumpZoneTrigger)
        .onFalse(
            Commands.runOnce(
                () ->
                    currentDriveMode =
                        (hubTrackTrigger.getAsBoolean()
                            ? DriveMode.TARGET_TRACK
                            : DriveMode.NORMAL)));

    RobotContainer.getTrackTrigger()
        .onTrue(Commands.runOnce(() -> forceTargetTracking = !forceTargetTracking));

    hubTrackTrigger
        .onTrue(updateDriveMode(DriveMode.TARGET_TRACK))
        .onFalse(updateDriveMode(DriveMode.NORMAL));

    if (hubAimTrigger != null) {
      hubAimTrigger
          .whileTrue(updateDriveMode(DriveMode.TARGET_AIM))
          .onFalse(
              Commands.runOnce(
                  () ->
                      currentDriveMode =
                          hubTrackTrigger.getAsBoolean()
                              ? DriveMode.TARGET_TRACK
                              : DriveMode.NORMAL));
    }

    for (int i = 0; i < 4; i++) {
      Logger.recordOutput("Trench" + i, TeleopDriveConstants.TRENCH_ZONES[i]);
      Logger.recordOutput("Bump" + i, TeleopDriveConstants.BUMP_ZONES[i]);
    }
    addRequirements(drive);
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), ControllerConstants.DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));
    linearMagnitude = linearMagnitude * linearMagnitude;

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

  @Override
  public void initialize() {
    flipFactor =
        DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red
            ? -1
            : 1;
  }

  @Override
  public void execute() {
    // Forcing the drive to X
    boolean shouldStopWithX =
        forceTargetLockThenX && ShooterCommandsUtil.inPositionToShoot(drive, targetSupplier.get());
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
        omega = Math.copySign(omega * omega, omega);

        drive.driveFieldCentric(
            MetersPerSecond.of(linearVelocity.getX()),
            MetersPerSecond.of(linearVelocity.getY()),
            maxRotSpeed.times(omega));
        break;

      case TARGET_TRACK:
        rotSpeedToTarget =
            rotationController.calculate(
                drive.getRotation().getRadians(),
                ShooterCommandsUtil.getTargetLockAngle(drive, targetSupplier.get()).getRadians());
        drive.driveFieldCentric(
            MetersPerSecond.of(linearVelocity.getX()),
            MetersPerSecond.of(linearVelocity.getY()),
            RadiansPerSecond.of(rotSpeedToTarget));
        break;

      case TARGET_AIM:
        rotSpeedToTarget =
            rotationController.calculate(
                drive.getRotation().getRadians(),
                ShooterCommandsUtil.getTargetLockAngle(drive, targetSupplier.get()).getRadians());
        drive.driveFieldCentric(
            MetersPerSecond.of(0), MetersPerSecond.of(0), RadiansPerSecond.of(rotSpeedToTarget));
        break;
    }
  }

  private void setSpeed(DriveSpeed newDriveSpeed) {
    driveSpeed = newDriveSpeed;
    switch (newDriveSpeed) {
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

  private void enableTargetLock(Supplier<Translation3d> supplier, boolean lockThenX) {
    targetSupplier = supplier;
    forceTargetTracking = true;
    forceTargetLockThenX = lockThenX;
    currentDriveMode = DriveMode.TARGET_TRACK;
  }

  private void disableTargetLock() {
    forceTargetTracking = false;
    forceTargetLockThenX = false;
    currentDriveMode = DriveMode.NORMAL;
    targetSupplier = HUB_TARGET_SUPPLIER;
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

  public Command withSpeed(DriveSpeed speed) {
    return Commands.startEnd(() -> setSpeed(speed), () -> setSpeed(DriveSpeed.DEFAULT));
  }

  public Command withHubLock() {
    return withTargetLock(HUB_TARGET_SUPPLIER);
  }

  public Command withHubLockThenX() {
    return withTargetLockThenX(HUB_TARGET_SUPPLIER);
  }

  public Command withTargetLock(Supplier<Translation3d> supplier) {
    return Commands.startEnd(() -> enableTargetLock(supplier, false), this::disableTargetLock);
  }

  public Command withTargetLockThenX(Supplier<Translation3d> supplier) {
    return Commands.startEnd(() -> enableTargetLock(supplier, true), this::disableTargetLock);
  }

  public Command withHubLock(Command command) {
    return withTargetLock(HUB_TARGET_SUPPLIER, command);
  }

  public Command withHubLockThenX(Command command) {
    return withTargetLockThenX(HUB_TARGET_SUPPLIER, command);
  }

  public Command withTargetLock(Supplier<Translation3d> supplier, Command command) {
    return command.alongWith(withTargetLock(supplier));
  }

  public Command withTargetLockThenX(Supplier<Translation3d> supplier, Command command) {
    return command.alongWith(withTargetLockThenX(supplier));
  }

  public Command withSpeed(DriveSpeed speed, Command command) {
    return command.alongWith(withSpeed(speed));
  }

  private enum DriveMode {
    NORMAL,
    TRENCH_LOCK,
    BUMP_LOCK,
    TARGET_TRACK,
    TARGET_AIM
  }
}
