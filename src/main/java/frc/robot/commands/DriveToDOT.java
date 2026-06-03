// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.AkitDrive;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TunableControls.ControlConstants;
import frc.robot.util.TunableControls.TunableControlConstants;
import frc.robot.util.TunableControls.TunablePIDController;
import java.util.function.Supplier;
import org.littletonrobotics.frc2026.util.geometry.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

public class DriveToDOT extends Command {

  public enum Side {
    LEFT,
    CENTER,
    RIGHT
  }

  // Seconds into autonomous at which the robot starts driving to the DOT.
  // Set to 0 to drive immediately. Editable from Elastic under /Tuning/.
  private static final LoggedTunableNumber waitUntilAutoSeconds =
      new LoggedTunableNumber("DriveToDOT/WaitUntilAutoSeconds", 0.0);

  // Distance from the DOT centre at which X-lock engages / disengages.
  private static final LoggedTunableNumber holdToleranceInches =
      new LoggedTunableNumber("DriveToDOT/HoldToleranceInches", 3.0);

  // Robot centre must stay within this distance of the DOT centre to be considered "covering" it.
  // Approximation: robot is 32x32 in, DOT is 12 in diameter → robot edge reaches ~16 in from its
  // centre, so 16 in is a reasonable default.
  private static final LoggedTunableNumber coveringToleranceInches =
      new LoggedTunableNumber("DriveToDOT/CoveringToleranceInches", 16.0);

  // If the robot has not been covering the target DOT for this many seconds, fall back to CENTER.
  private static final LoggedTunableNumber notCoveringTimeoutSeconds =
      new LoggedTunableNumber("DriveToDOT/NotCoveringTimeoutSeconds", 3.0);

  private static final double AUTO_PERIOD_SECONDS = 20.0;
  private static final double POSITION_MAX_VELOCITY_MPS = 2.0;
  private static final double ROTATION_MAX_VELOCITY_DEGPS = 720.0;

  private final TunablePIDController positionXController;
  private final TunablePIDController positionYController;
  private final TunablePIDController rotationController;

  private final AkitDrive drive;
  private final Supplier<Side> sideSupplier;

  private Pose2d targetPose;
  private Side activeSide;
  private boolean holding;
  private final Timer notCoveringTimer = new Timer();

  public DriveToDOT(Supplier<Side> sideSupplier, AkitDrive drive) {
    this.drive = drive;
    this.sideSupplier = sideSupplier;

    TunableControlConstants positionConstants =
        new TunableControlConstants(
            "DriveToDOT/PositionConstants",
            new ControlConstants().withPID(2.0, 0, 0).withTolerance(0.05));
    TunableControlConstants rotationConstants =
        new TunableControlConstants(
            "DriveToDOT/RotationConstants",
            new ControlConstants()
                .withPID(0.5, 0, 0)
                .withContinuous(-Math.PI, Math.PI)
                .withTolerance(Math.toRadians(3)));

    positionXController = new TunablePIDController(positionConstants);
    positionYController = new TunablePIDController(positionConstants);
    rotationController = new TunablePIDController(rotationConstants);

    addRequirements(drive);
    setName("DriveToDOT");
  }

  @Override
  public void initialize() {
    activeSide = sideSupplier.get();
    setTarget(activeSide);

    holding = false;
    notCoveringTimer.stop();
    notCoveringTimer.reset();

    positionXController.reset();
    positionYController.reset();
    rotationController.reset();
  }

  @Override
  public void execute() {
    // --- Wait / park phase: X-lock until the auto time threshold is reached ---
    double matchTime = DriverStation.getMatchTime();
    if (matchTime >= 0 && (AUTO_PERIOD_SECONDS - matchTime) < waitUntilAutoSeconds.get()) {
      notCoveringTimer.stop();
      notCoveringTimer.reset();
      drive.stopWithX();
      return;
    }

    // --- Active phase ---
    Pose2d currentPose = drive.getPose();
    double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    double holdThresholdMeters = Inches.of(holdToleranceInches.get()).in(Meters);
    double coveringThresholdMeters = Inches.of(coveringToleranceInches.get()).in(Meters);

    // --- Coverage tracking: fall back to CENTER if away too long ---
    if (distance <= coveringThresholdMeters) {
      notCoveringTimer.stop();
      notCoveringTimer.reset();
    } else {
      notCoveringTimer.start();
      if (activeSide != Side.CENTER
          && notCoveringTimer.hasElapsed(notCoveringTimeoutSeconds.get())) {
        activeSide = Side.CENTER;
        setTarget(Side.CENTER);
        holding = false;
        notCoveringTimer.stop();
        notCoveringTimer.reset();
        positionXController.reset();
        positionYController.reset();
        rotationController.reset();
        return; // let the next cycle drive toward the new target
      }
    }

    Logger.recordOutput("DriveToDOT/CurrentPose", currentPose);
    Logger.recordOutput("DriveToDOT/DistanceToTargetMeters", distance);
    Logger.recordOutput("DriveToDOT/Holding", holding);
    Logger.recordOutput("DriveToDOT/ActiveSide", activeSide.toString());

    // --- X-lock or PID drive ---
    if (distance <= holdThresholdMeters) {
      holding = true;
      drive.stopWithX();
    } else {
      if (holding) {
        // Pushed out of hold zone — reset integrators before resuming drive
        positionXController.reset();
        positionYController.reset();
        rotationController.reset();
        holding = false;
      }

      double calcX = positionXController.calculate(currentPose.getX(), targetPose.getX());
      double calcY = positionYController.calculate(currentPose.getY(), targetPose.getY());
      double calcTheta =
          rotationController.calculate(
              currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

      drive.driveFieldCentric(
          MetersPerSecond.of(POSITION_MAX_VELOCITY_MPS * calcX),
          MetersPerSecond.of(POSITION_MAX_VELOCITY_MPS * calcY),
          DegreesPerSecond.of(ROTATION_MAX_VELOCITY_DEGPS * calcTheta));
    }
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // runs until interrupted — robot keeps holding position
  }

  private void setTarget(Side side) {
    Translation2d translation =
        side == Side.LEFT
            ? TeleopDriveConstants.BC_DOTS[0]
            : side == Side.CENTER
                ? TeleopDriveConstants.BC_DOTS[1]
                : TeleopDriveConstants.BC_DOTS[2];
    targetPose = AllianceFlipUtil.apply(new Pose2d(translation, Rotation2d.fromDegrees(180)));
    Logger.recordOutput("DriveToDOT/TargetPose", targetPose);
  }
}
