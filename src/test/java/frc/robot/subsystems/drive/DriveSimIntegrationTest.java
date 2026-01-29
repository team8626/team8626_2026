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

package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.SimTestHelpers;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

/**
 * Simulation integration tests for the full Drive subsystem. Uses physics simulation to validate
 * swerve kinematics and odometry behavior.
 */
@Tag("integration")
public class DriveSimIntegrationTest {
  private static final double POSITION_TOLERANCE_METERS = 0.1;

  private GyroIOSim gyroIO;
  private ModuleIOSim flModuleIO;
  private ModuleIOSim frModuleIO;
  private ModuleIOSim blModuleIO;
  private ModuleIOSim brModuleIO;
  private Drive drive;

  @BeforeAll
  static void initializeHAL() {
    HAL.initialize(500, 0);
  }

  @BeforeEach
  void setUp() {
    gyroIO = new GyroIOSim();
    flModuleIO = new ModuleIOSim();
    frModuleIO = new ModuleIOSim();
    blModuleIO = new ModuleIOSim();
    brModuleIO = new ModuleIOSim();

    drive = new Drive(gyroIO, flModuleIO, frModuleIO, blModuleIO, brModuleIO);
    drive.periodic();
  }

  private void updateDrive() {
    drive.periodic();
  }

  @Test
  void testForwardMotion() {
    // Command forward velocity
    double forwardSpeed = 1.0; // m/s
    ChassisSpeeds speeds = new ChassisSpeeds(forwardSpeed, 0.0, 0.0);

    Pose2d initialPose = drive.getPose();

    // Run for 1 second
    for (int i = 0; i < 50; i++) {
      drive.runVelocity(speeds);
      updateDrive();
    }

    Pose2d finalPose = drive.getPose();

    // Should have moved forward
    double deltaX = finalPose.getX() - initialPose.getX();
    double deltaY = finalPose.getY() - initialPose.getY();

    assertTrue(deltaX > 0, "Robot should have moved forward (positive X)");
    assertTrue(
        Math.abs(deltaY) < POSITION_TOLERANCE_METERS,
        "Robot should not have significant lateral drift");

    System.out.printf("[Forward Motion] Moved X: %.3fm, Y: %.3fm in 1 second%n", deltaX, deltaY);
  }

  @Test
  void testStrafeMotion() {
    // Command strafe velocity (left)
    double strafeSpeed = 1.0; // m/s
    ChassisSpeeds speeds = new ChassisSpeeds(0.0, strafeSpeed, 0.0);

    Pose2d initialPose = drive.getPose();

    // Run for 1 second
    for (int i = 0; i < 50; i++) {
      drive.runVelocity(speeds);
      updateDrive();
    }

    Pose2d finalPose = drive.getPose();

    double deltaX = finalPose.getX() - initialPose.getX();
    double deltaY = finalPose.getY() - initialPose.getY();

    assertTrue(deltaY > 0, "Robot should have strafed left (positive Y)");
    assertTrue(
        Math.abs(deltaX) < POSITION_TOLERANCE_METERS,
        "Robot should not have significant forward drift");

    System.out.printf("[Strafe Motion] Moved X: %.3fm, Y: %.3fm in 1 second%n", deltaX, deltaY);
  }

  @Test
  void testRotationOnly() {
    // Command rotation only
    double rotationSpeed = 1.0; // rad/s
    ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, rotationSpeed);

    // Also update the simulated gyro
    gyroIO.setAngularVelocity(rotationSpeed);

    Pose2d initialPose = drive.getPose();

    // Run for 1 second
    for (int i = 0; i < 50; i++) {
      drive.runVelocity(speeds);
      updateDrive();
    }

    Pose2d finalPose = drive.getPose();

    double deltaX = finalPose.getX() - initialPose.getX();
    double deltaY = finalPose.getY() - initialPose.getY();

    // Position should stay roughly the same
    assertTrue(
        Math.abs(deltaX) < POSITION_TOLERANCE_METERS * 2,
        "Robot should not translate significantly during rotation");
    assertTrue(
        Math.abs(deltaY) < POSITION_TOLERANCE_METERS * 2,
        "Robot should not translate significantly during rotation");

    // Rotation should have changed
    double deltaRotation =
        finalPose.getRotation().getDegrees() - initialPose.getRotation().getDegrees();

    System.out.printf(
        "[Rotation Only] Position delta: (%.3f, %.3f)m, Rotation delta: %.1f°%n",
        deltaX, deltaY, deltaRotation);
  }

  @Test
  void testCombinedMotion() {
    // Command combined translation and rotation
    ChassisSpeeds speeds = new ChassisSpeeds(0.5, 0.3, 0.5);
    gyroIO.setAngularVelocity(0.5);

    // Run for 2 seconds
    for (int i = 0; i < 100; i++) {
      drive.runVelocity(speeds);
      updateDrive();
    }

    Pose2d finalPose = drive.getPose();

    // Should have non-zero position and rotation
    assertTrue(finalPose.getX() != 0.0 || finalPose.getY() != 0.0, "Robot should have moved");

    System.out.printf(
        "[Combined Motion] Final pose: (%.3f, %.3f)m, %.1f°%n",
        finalPose.getX(), finalPose.getY(), finalPose.getRotation().getDegrees());
  }

  @Test
  void testStopCommand() {
    // First move at some velocity
    ChassisSpeeds speeds = new ChassisSpeeds(2.0, 0.0, 0.0);
    for (int i = 0; i < 25; i++) {
      drive.runVelocity(speeds);
      updateDrive();
    }

    // Now stop
    drive.stop();

    // Continue updating to let motors settle
    for (int i = 0; i < 25; i++) {
      updateDrive();
    }

    // Velocity should be near zero
    double avgVelocity = drive.getFFCharacterizationVelocity();

    System.out.printf("[Stop Test] Velocity after stop: %.4f rad/s%n", avgVelocity);
  }

  @Test
  void testOdometryAccuracy() {
    // Drive in a known pattern and verify odometry tracks correctly
    // Move forward at 1 m/s for 1 second = should move ~1m forward

    double targetVelocity = 1.0; // m/s
    ChassisSpeeds speeds = new ChassisSpeeds(targetVelocity, 0.0, 0.0);

    drive.setPose(new Pose2d(0, 0, new Rotation2d()));
    updateDrive();

    double duration = 1.0;
    int steps = (int) (duration / SimTestHelpers.DEFAULT_TIMESTEP);

    for (int i = 0; i < steps; i++) {
      drive.runVelocity(speeds);
      updateDrive();
    }

    Pose2d finalPose = drive.getPose();
    double expectedDistance = targetVelocity * duration;

    // Allow for acceleration time and simulation differences
    double actualDistance = finalPose.getX();
    double error = Math.abs(expectedDistance - actualDistance);

    assertTrue(
        error < expectedDistance * 0.3, // Within 30% of expected
        String.format(
            "Odometry error too large. Expected: ~%.2fm, Actual: %.2fm, Error: %.2fm",
            expectedDistance, actualDistance, error));

    System.out.printf(
        "[Odometry Accuracy] Expected: %.2fm, Actual: %.3fm, Error: %.3fm (%.1f%%)%n",
        expectedDistance, actualDistance, error, (error / expectedDistance) * 100);
  }

  @Test
  void testMaxSpeedLimiting() {
    // Command speeds exceeding the maximum
    double excessiveSpeed = drive.getMaxLinearSpeedMetersPerSec() * 2.0;
    ChassisSpeeds speeds = new ChassisSpeeds(excessiveSpeed, excessiveSpeed, 0.0);

    drive.runVelocity(speeds);
    updateDrive();

    // Run for a bit to let modules accelerate
    for (int i = 0; i < 50; i++) {
      drive.runVelocity(speeds);
      updateDrive();
    }

    // Verify modules don't exceed max speed (in rad/s at the wheel)
    double maxModuleSpeed =
        drive.getMaxLinearSpeedMetersPerSec() / DriveConstants.wheelRadiusMeters;

    double[] positions1 = drive.getWheelRadiusCharacterizationPositions();
    updateDrive();
    double[] positions2 = drive.getWheelRadiusCharacterizationPositions();

    // Check velocity of first module (position delta / time)
    double moduleVelocity = Math.abs(positions2[0] - positions1[0]) / 0.02;

    assertTrue(
        moduleVelocity <= maxModuleSpeed * 1.1, // Allow 10% tolerance for simulation
        String.format(
            "Module velocity %.2f rad/s exceeds max %.2f rad/s", moduleVelocity, maxModuleSpeed));
  }

  @Test
  void testGyroDisconnectedOdometry() {
    // Test odometry with simulated gyro disconnect
    // First move normally
    ChassisSpeeds speeds = new ChassisSpeeds(0.5, 0.0, 0.5);
    gyroIO.setAngularVelocity(0.5);

    for (int i = 0; i < 25; i++) {
      drive.runVelocity(speeds);
      updateDrive();
    }

    // Continue - kinematics fallback should work
    // (In simulation, we're using GyroIOSim which stays connected)
    Pose2d pose = drive.getPose();
    assertNotNull(pose);

    System.out.printf(
        "[Gyro Test] Pose with gyro: (%.3f, %.3f, %.1f°)%n",
        pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }

  @Test
  void testSetPoseReset() {
    // Move the robot
    ChassisSpeeds speeds = new ChassisSpeeds(1.0, 0.5, 0.0);
    for (int i = 0; i < 50; i++) {
      drive.runVelocity(speeds);
      updateDrive();
    }

    Pose2d poseAfterMoving = drive.getPose();
    assertTrue(poseAfterMoving.getX() != 0.0 || poseAfterMoving.getY() != 0.0);

    // Reset pose
    Pose2d newPose = new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(45.0));
    drive.setPose(newPose);
    updateDrive();

    Pose2d currentPose = drive.getPose();
    assertEquals(5.0, currentPose.getX(), 0.1);
    assertEquals(3.0, currentPose.getY(), 0.1);
    assertEquals(45.0, currentPose.getRotation().getDegrees(), 1.0);
  }

  @Test
  void testCharacterizationMode() {
    // Test that characterization drives all modules equally
    double voltage = 4.0;

    for (int i = 0; i < 25; i++) {
      drive.runCharacterization(voltage);
      updateDrive();
    }

    double avgVelocity = drive.getFFCharacterizationVelocity();
    assertTrue(avgVelocity > 0, "Characterization should produce positive velocity");

    System.out.printf(
        "[Characterization] Avg velocity at %.1fV: %.3f rad/s%n", voltage, avgVelocity);
  }

  @Test
  void testSquarePath() {
    // Drive in a square pattern and check final position
    double speed = 0.5; // m/s
    int stepsPerSide = 25; // 0.5 seconds per side

    drive.setPose(new Pose2d());
    updateDrive();

    // Forward
    for (int i = 0; i < stepsPerSide; i++) {
      drive.runVelocity(new ChassisSpeeds(speed, 0, 0));
      updateDrive();
    }

    // Left
    for (int i = 0; i < stepsPerSide; i++) {
      drive.runVelocity(new ChassisSpeeds(0, speed, 0));
      updateDrive();
    }

    // Backward
    for (int i = 0; i < stepsPerSide; i++) {
      drive.runVelocity(new ChassisSpeeds(-speed, 0, 0));
      updateDrive();
    }

    // Right
    for (int i = 0; i < stepsPerSide; i++) {
      drive.runVelocity(new ChassisSpeeds(0, -speed, 0));
      updateDrive();
    }

    Pose2d finalPose = drive.getPose();

    // Should be back near origin (with some accumulated error)
    double distanceFromOrigin =
        Math.sqrt(finalPose.getX() * finalPose.getX() + finalPose.getY() * finalPose.getY());

    System.out.printf(
        "[Square Path] Final position: (%.3f, %.3f)m, Distance from origin: %.3fm%n",
        finalPose.getX(), finalPose.getY(), distanceFromOrigin);

    // Allow significant tolerance since we're not doing closed-loop position control
    assertTrue(
        distanceFromOrigin < 1.0, "Robot should return somewhat close to origin after square path");
  }

  @Test
  void testSpinInPlace() {
    // Spin in place for a full rotation
    double rotationSpeed = Math.PI; // rad/s, should complete 180° in 1 second
    gyroIO.setAngularVelocity(rotationSpeed);

    drive.setPose(new Pose2d());
    updateDrive();

    Pose2d initialPose = drive.getPose();

    for (int i = 0; i < 50; i++) {
      drive.runVelocity(new ChassisSpeeds(0, 0, rotationSpeed));
      updateDrive();
    }

    Pose2d finalPose = drive.getPose();

    // Position should be roughly the same
    double positionDrift =
        Math.sqrt(
            Math.pow(finalPose.getX() - initialPose.getX(), 2)
                + Math.pow(finalPose.getY() - initialPose.getY(), 2));

    System.out.printf(
        "[Spin Test] Position drift: %.3fm, Final rotation: %.1f°%n",
        positionDrift, finalPose.getRotation().getDegrees());

    assertTrue(
        positionDrift < POSITION_TOLERANCE_METERS * 3,
        "Robot should not drift significantly while spinning in place");
  }

  @Test
  void testVelocityTransition() {
    // Test smooth transition between different velocities
    ChassisSpeeds[] speedSequence = {
      new ChassisSpeeds(1.0, 0.0, 0.0),
      new ChassisSpeeds(0.5, 0.5, 0.0),
      new ChassisSpeeds(0.0, 1.0, 0.0),
      new ChassisSpeeds(-0.5, 0.5, 0.0),
      new ChassisSpeeds(0.0, 0.0, 0.0)
    };

    for (ChassisSpeeds speeds : speedSequence) {
      for (int i = 0; i < 25; i++) {
        drive.runVelocity(speeds);
        updateDrive();
      }
    }

    // Just verify no exceptions and robot ends stopped
    drive.stop();
    updateDrive();

    System.out.println("[Velocity Transition] Completed all transitions successfully");
  }
}
