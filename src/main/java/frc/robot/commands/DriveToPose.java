// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.AkitDrive;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TunableControls.ControlConstants;
import frc.robot.util.TunableControls.TunableControlConstants;
import frc.robot.util.TunableControls.TunablePIDController;
import java.util.function.Supplier;
import org.littletonrobotics.frc2026.util.geometry.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {

  private final AkitDrive m_drive;

  private final LoggedTunableNumber positionTolerance =
      new LoggedTunableNumber("DriveToPose/TolerancePostionMeters", 0.05);
  private final LoggedTunableNumber rotationTolerance =
      new LoggedTunableNumber("DriveToPose/ToleranceRotationRadians", Degrees.of(3).in(Radians));

  private final ControlConstants POSITION_CONSTANTS =
      new ControlConstants().withPID(0.5, 0, 0).withTolerance(positionTolerance.get());
  private final ControlConstants ROTATION_CONSTANTS =
      new ControlConstants()
          .withPID(0.5, 0, 0)
          .withContinuous(-Math.PI, Math.PI)
          .withTolerance(rotationTolerance.get());

  private final TunableControlConstants controllerPositionConstants =
      new TunableControlConstants("DrivetToPose/PositionConstants", POSITION_CONSTANTS);
  private final TunableControlConstants controllerRotationConstants =
      new TunableControlConstants("DrivetToPose/RotationConstants", ROTATION_CONSTANTS);

  private final TunablePIDController positionXController =
      new TunablePIDController(controllerPositionConstants);
  private final TunablePIDController positionYController =
      new TunablePIDController(controllerPositionConstants);
  private final TunablePIDController rotationController =
      new TunablePIDController(controllerRotationConstants);

  private Supplier<Pose2d> desiredPose;
  private Pose2d m_pose;

  private double m_xDesiredPos;
  private double m_yDesiredPos;
  private double m_desiredRotRadians;

  private LinearVelocity positionMaxVelocity = MetersPerSecond.of(2);
  //   private LinearAcceleration positionMaxAcceleration = MetersPerSecondPerSecond.of(2);

  private AngularVelocity rotationMaxVelocity = DegreesPerSecond.of(720);
  //   private AngularAcceleration rotationMaxAcceleration = DegreesPerSecondPerSecond.of(360);

  private static final Distance defaultPositionTolerance = Meters.of(0.025);
  private static final Distance defaultPoseOffsetX = Inches.of(0);
  private static final Angle defaultRotationTolerance = Degrees.of(2.0);
  private boolean hasValidPose = false;

  private Distance positionOffsetX = Inches.of(0);

  public DriveToPose(Supplier<Pose2d> desiredPoseSupplier, AkitDrive drive) {
    this(
        desiredPoseSupplier,
        () -> defaultPoseOffsetX,
        () -> defaultPositionTolerance,
        () -> defaultRotationTolerance,
        drive);
  }

  public DriveToPose(
      Supplier<Pose2d> desiredPoseSupplier, Supplier<Distance> offset, AkitDrive drive) {
    this(
        desiredPoseSupplier,
        offset,
        () -> defaultPositionTolerance,
        () -> defaultRotationTolerance,
        drive);
  }

  public DriveToPose(
      Supplier<Pose2d> desiredPoseSupplier,
      Supplier<Distance> new_positionTolerance,
      Supplier<Angle> new_rotationTolerance,
      AkitDrive drive) {
    this(
        desiredPoseSupplier,
        () -> defaultPoseOffsetX,
        () -> defaultPositionTolerance,
        () -> defaultRotationTolerance,
        drive);
  }
  /**
   * Creates a new DriveToPoseFinkle command.
   *
   * @param new_pose
   * @param offset as a Distance
   * @param posToleranceSupplier as a Distance
   * @param new_rotationTolerance as an Angle
   */
  public DriveToPose(
      Supplier<Pose2d> new_pose,
      Supplier<Distance> offset,
      Supplier<Distance> new_positionTolerance,
      Supplier<Angle> new_rotationTolerance,
      AkitDrive drive) {
    m_drive = drive;

    positionOffsetX = offset.get();
    desiredPose = () -> AllianceFlipUtil.apply(new_pose.get());

    setName("DriveToPose");

    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    hasValidPose = false;
    if (desiredPose != null) {
      if (desiredPose.get() != null) {
        hasValidPose = true;
      }
    }

    if (hasValidPose == true) {
      m_pose = m_drive.getPose();
      updatePositionValues();

      Pose2d offsetPose =
          desiredPose.get().plus(new Transform2d(positionOffsetX.in(Meters), 0, new Rotation2d()));

      Logger.recordOutput("DriveToPose/OffsetPose", offsetPose);
      Logger.recordOutput("DriveToPose/CurrentPose", m_pose);

      m_xDesiredPos = offsetPose.getX();
      m_yDesiredPos = offsetPose.getY();
      m_desiredRotRadians = offsetPose.getRotation().getRadians();

      // Reset the PID controllers
      positionXController.reset();
      positionYController.reset();
      rotationController.reset();
    }
  }

  @Override
  public void execute() {
    if (hasValidPose) {
      m_pose = m_drive.getPose();

      Logger.recordOutput("DriveToPose/TargetPose", desiredPose.get());
      Logger.recordOutput("DriveToPose/CurrentPose", m_pose);

      double calculateX = positionXController.calculate(m_pose.getX(), m_xDesiredPos);
      double calculateY = positionYController.calculate(m_pose.getY(), m_yDesiredPos);
      double calculateTheta =
          rotationController.calculate(m_pose.getRotation().getRadians(), m_desiredRotRadians);

      Logger.recordOutput("DriveToPose/CalculateX", calculateX);
      Logger.recordOutput("DriveToPose/CalculateY", calculateY);
      Logger.recordOutput("DriveToPose/CalculateTheta", calculateTheta);

      LinearVelocity linearVelocityX = positionMaxVelocity.times(calculateX);
      LinearVelocity linearVelocityY = positionMaxVelocity.times(calculateY);
      AngularVelocity angularVelocity = rotationMaxVelocity.times(calculateTheta);
      m_drive.driveFieldCentric(linearVelocityX, linearVelocityY, angularVelocity);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return hasValidPose
        && (positionXController.atSetpoint()
            && positionYController.atSetpoint()
            && rotationController.atSetpoint());
    // && (xPIDController.atSetpoint()
    // && yPIDController.atSetpoint()
    // && rotPIDController.atSetpoint());
  }

  private void updateTolerancesValues() {
    // Distance newPositionTolerance =
    //     Meters.of(
    //         SmartDashboard.getNumber(
    //             "Commands/DriveToPoseFinkle/PositionTolerance(m)",
    // positionTolerance.in(Meters)));
    // Angle newRotationTolerance =
    //     Degrees.of(
    //         SmartDashboard.getNumber(
    //             "Commands/DriveToPoseFinkle/RotationTolerance(deg)",
    //             rotationTolerance.in(Degrees)));

    // // Set the tolerances
    // xPIDController.setTolerance(
    //     newPositionTolerance.in(Meters), defaultPositionVelocityTolerance.in(MetersPerSecond));
    // yPIDController.setTolerance(
    //     newPositionTolerance.in(Meters), defaultPositionVelocityTolerance.in(MetersPerSecond));
    // rotPIDController.setTolerance(
    //     newRotationTolerance.in(Radians), defaultRotationVelocityTolerance.in(RadiansPerSecond));
  }

  private void updatePositionValues() {
    if (SmartDashboard.getBoolean("Commands/DriveToPoseFinkle/OverrideOffsetDistance", false)) {
      positionOffsetX =
          Inches.of(SmartDashboard.getNumber("Commands/DriveToPoseFinkle/OffsetDistance(in)", 7.0));
    }
  }
}
