package frc.robot.commands;

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
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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

/** Default drive command to run that drives based on controller input. */
public class TeleopDriveCommand extends Command {
  private static final Supplier<Translation3d> HUB_TARGET_SUPPLIER =
      () -> FieldConstants.Hub.topCenterPoint;

  private final AkitDrive drive;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier omegaSupplier;
  private final SlewRateLimiter2d driveLimiter;
  private final TunablePIDController rotationController =
      new TunablePIDController(DriveConstants.ROTATION_CONSTANTS);

  private LinearVelocity maxDriveSpeed = DriveConstants.DEFAULT_DRIVE_SPEED;
  private AngularVelocity maxRotSpeed = DriveConstants.DEFAULT_ROT_SPEED;

  @AutoLogOutput(key = "Drive/SpeedMode")
  private DriveSpeed driveSpeed = DriveSpeed.DEFAULT;

  /** Persistent driver toggle. */
  @AutoLogOutput(key = "Drive/ManualTargetTracking")
  private boolean manualTargetTracking = false;

  /** Temporary command-owned state. */
  @AutoLogOutput(key = "Drive/CommandTargetTracking")
  private boolean commandTargetTracking = false;

  private boolean commandLockThenX = false;
  private Supplier<Translation3d> commandTargetSupplier = HUB_TARGET_SUPPLIER;

  @AutoLogOutput(key = "Drive/RotSpeedToTargetRadPerSec")
  private double rotSpeedToTarget = 0.0;

  public TeleopDriveCommand(AkitDrive drive, CommandXboxController controller) {
    this.drive = drive;
    this.xSupplier = () -> -controller.getLeftY() * getAllianceFlipFactor();
    this.ySupplier = () -> -controller.getLeftX() * getAllianceFlipFactor();
    this.omegaSupplier = () -> -controller.getRightX();
    this.driveLimiter =
        new SlewRateLimiter2d(DriveConstants.MAX_TELEOP_ACCEL.in(MetersPerSecondPerSecond));

    RobotContainer.getTrackTrigger()
        .onTrue(Commands.runOnce(() -> manualTargetTracking = !manualTargetTracking));

    addRequirements(drive);
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), ControllerConstants.DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    linearMagnitude *= linearMagnitude;

    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  private int getAllianceFlipFactor() {
    return DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red
        ? -1
        : 1;
  }

  private DriveMode getDriveMode() {
    if (RobotContainer.getAimTrigger().getAsBoolean()) {
      return DriveMode.TARGET_AIM;
    }
    if (manualTargetTracking || commandTargetTracking) {
      return DriveMode.TARGET_TRACK;
    }
    return DriveMode.NORMAL;
  }

  @AutoLogOutput(key = "Drive/CurrentDriveMode")
  private String getCurrentDriveModeLog() {
    return getDriveMode().name();
  }

  @AutoLogOutput(key = "Drive/XLockActive")
  private boolean getXLockActiveLog() {
    return isXLockActive();
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Supplier<Translation3d> activeTargetSupplier =
        commandTargetTracking ? commandTargetSupplier : () -> FieldConstants.Hub.topCenterPoint;

    if (isXLockActive()) {
      rotSpeedToTarget = 0.0;
      drive.stopWithX();
      return;
    }

    Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
    linearVelocity = linearVelocity.times(maxDriveSpeed.in(MetersPerSecond));
    linearVelocity = driveLimiter.calculate(linearVelocity);

    switch (getDriveMode()) {
      case NORMAL -> {
        rotSpeedToTarget = 0.0;

        double omega =
            MathUtil.applyDeadband(omegaSupplier.getAsDouble(), ControllerConstants.DEADBAND);
        omega = Math.copySign(omega * omega, omega);

        drive.driveFieldCentric(
            MetersPerSecond.of(linearVelocity.getX()),
            MetersPerSecond.of(linearVelocity.getY()),
            maxRotSpeed.times(omega));
      }

      case TARGET_TRACK -> {
        rotSpeedToTarget =
            rotationController.calculate(
                drive.getRotation().getRadians(),
                ShooterCommandsUtil.getTargetLockAngle(drive, activeTargetSupplier.get())
                    .getRadians());

        drive.driveFieldCentric(
            MetersPerSecond.of(linearVelocity.getX()),
            MetersPerSecond.of(linearVelocity.getY()),
            RadiansPerSecond.of(rotSpeedToTarget));
      }

      case TARGET_AIM -> {
        rotSpeedToTarget =
            rotationController.calculate(
                drive.getRotation().getRadians(),
                ShooterCommandsUtil.getTargetLockAngle(drive, activeTargetSupplier.get())
                    .getRadians());

        drive.driveFieldCentric(
            MetersPerSecond.zero(), MetersPerSecond.zero(), RadiansPerSecond.of(rotSpeedToTarget));
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    rotSpeedToTarget = 0.0;
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  // -----------------------------------------------------------------
  // Helper methods to apply drive modifiers to other commands.
  public Command withSpeed(DriveSpeed speed) {
    return Commands.startEnd(() -> setSpeed(speed), () -> setSpeed(DriveSpeed.DEFAULT));
  }

  public Command withSpeed(DriveSpeed speed, Command command) {
    return Commands.parallel(command, withSpeed(speed));
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

  // -----------------------------------------------------------------
  // Helper methods to apply target locks to other commands.
  public Command withHubLock() {
    return withTargetLock(HUB_TARGET_SUPPLIER);
  }

  public Command withHubLock(Command command) {
    return withTargetLock(HUB_TARGET_SUPPLIER, command);
  }

  private Command withTargetLock(Supplier<Translation3d> supplier) {
    return Commands.startEnd(
        () -> enableCommandTargetLock(supplier, false), this::disableCommandTargetLock);
  }

  public Command withTargetLock(Supplier<Translation3d> supplier, Command command) {
    return Commands.parallel(command, withTargetLock(supplier));
  }

  public Command withTargetLockThenX(Supplier<Translation3d> supplier, Command command) {
    return Commands.parallel(command, withTargetLockThenX(supplier));
  }

  private void enableCommandTargetLock(Supplier<Translation3d> supplier, boolean lockThenX) {
    commandTargetSupplier = supplier;
    commandTargetTracking = true;
    commandLockThenX = lockThenX;
  }

  private void disableCommandTargetLock() {
    commandTargetTracking = false;
    commandLockThenX = false;
    commandTargetSupplier = HUB_TARGET_SUPPLIER;
  }

  // -----------------------------------------------------------------
  // Helper methods to apply target locks to other commands.
  // These methods will lock the drive to X position
  public Command withHubLockThenX() {
    return withTargetLockThenX(HUB_TARGET_SUPPLIER);
  }

  private Command withTargetLockThenX(Supplier<Translation3d> supplier) {
    return Commands.startEnd(
        () -> enableCommandTargetLock(supplier, true), this::disableCommandTargetLock);
  }

  public Command withHubLockThenX(Command command) {
    return withTargetLockThenX(HUB_TARGET_SUPPLIER, command);
  }

  private boolean isXLockActive() {
    return commandTargetTracking && commandLockThenX;
  }

  private enum DriveMode {
    NORMAL,
    TARGET_TRACK,
    TARGET_AIM
  }
}
