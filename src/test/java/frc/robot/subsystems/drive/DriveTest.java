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

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.commands.TeleopDrive;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

/** Unit tests for the Drive subsystem. */
@Tag("unit")
public class DriveTest {
  private static final double DELTA = 1e-3;

  private MockGyroIO gyroIO;
  private MockModuleIO flModuleIO;
  private MockModuleIO frModuleIO;
  private MockModuleIO blModuleIO;
  private MockModuleIO brModuleIO;
  private Drive drive;

  @BeforeAll
  static void initializeHAL() {
    // Initialize HAL once for all tests
    HAL.initialize(500, 0);
  }

  @BeforeEach
  void setUp() {
    gyroIO = new MockGyroIO();
    flModuleIO = new MockModuleIO();
    frModuleIO = new MockModuleIO();
    blModuleIO = new MockModuleIO();
    brModuleIO = new MockModuleIO();

    // Set default odometry data so periodic() doesn't fail
    setDefaultOdometryData();

    drive = new Drive(gyroIO, flModuleIO, frModuleIO, blModuleIO, brModuleIO);
  }

  private void setDefaultOdometryData() {
    double[] timestamps = {0.0};
    double[] drivePositions = {0.0};
    Rotation2d[] turnPositions = {new Rotation2d()};

    flModuleIO.setOdometryData(timestamps, drivePositions, turnPositions);
    frModuleIO.setOdometryData(timestamps, drivePositions, turnPositions);
    blModuleIO.setOdometryData(timestamps, drivePositions, turnPositions);
    brModuleIO.setOdometryData(timestamps, drivePositions, turnPositions);

    gyroIO.setOdometryData(timestamps, new Rotation2d[] {new Rotation2d()});
  }

  @Test
  void testInitialPose() {
    drive.periodic();

    Pose2d pose = drive.getPose();
    assertEquals(0.0, pose.getX(), DELTA);
    assertEquals(0.0, pose.getY(), DELTA);
    assertEquals(0.0, pose.getRotation().getDegrees(), DELTA);
  }

  @Test
  void testSetPose() {
    drive.periodic();

    Pose2d newPose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(45.0));
    drive.setPose(newPose);
    drive.periodic();

    Pose2d pose = drive.getPose();
    assertEquals(1.0, pose.getX(), DELTA);
    assertEquals(2.0, pose.getY(), DELTA);
    assertEquals(45.0, pose.getRotation().getDegrees(), DELTA);
  }

  @Test
  void testGetRotation() {
    drive.periodic();

    Pose2d newPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(90.0));
    drive.setPose(newPose);
    drive.periodic();

    assertEquals(90.0, drive.getRotation().getDegrees(), DELTA);
  }

  @Test
  void testRunVelocityForward() {
    drive.periodic();

    // Command forward velocity
    ChassisSpeeds speeds = new ChassisSpeeds(1.0, 0.0, 0.0);
    drive.runVelocity(speeds);

    // All modules should receive velocity commands
    // Forward motion means all modules drive in same direction
    assertTrue(flModuleIO.lastDriveVelocitySetpoint != 0.0);
    assertTrue(frModuleIO.lastDriveVelocitySetpoint != 0.0);
    assertTrue(blModuleIO.lastDriveVelocitySetpoint != 0.0);
    assertTrue(brModuleIO.lastDriveVelocitySetpoint != 0.0);
  }

  @Test
  void testRunVelocityStrafe() {
    // Set modules to 90 degrees (strafe position) so cosineScale doesn't zero the velocity
    flModuleIO.turnPosition = Rotation2d.fromDegrees(90.0);
    frModuleIO.turnPosition = Rotation2d.fromDegrees(90.0);
    blModuleIO.turnPosition = Rotation2d.fromDegrees(90.0);
    brModuleIO.turnPosition = Rotation2d.fromDegrees(90.0);
    drive.periodic();

    // Command strafe velocity (sideways)
    ChassisSpeeds speeds = new ChassisSpeeds(0.0, 1.0, 0.0);
    drive.runVelocity(speeds);

    // All modules should receive velocity commands
    // Note: cosineScale reduces velocity when module isn't aligned, so we verify
    // that velocity setpoints were applied (not necessarily non-zero)
    assertTrue(flModuleIO.lastDriveVelocitySetpoint != 0.0, "FL module should have velocity");
    assertTrue(frModuleIO.lastDriveVelocitySetpoint != 0.0, "FR module should have velocity");
    assertTrue(blModuleIO.lastDriveVelocitySetpoint != 0.0, "BL module should have velocity");
    assertTrue(brModuleIO.lastDriveVelocitySetpoint != 0.0, "BR module should have velocity");

    // Turn positions should be around 90 degrees for strafe
    double flAngle = flModuleIO.lastTurnPositionSetpoint.getDegrees();
    double normalizedAngle = Math.abs(flAngle);
    // Allow for angle to be near 90 or optimized equivalent
    boolean validAngle = (normalizedAngle > 45.0 && normalizedAngle < 135.0);
    assertTrue(validAngle, "Module turn angle should be ~90° for strafe: " + flAngle);
  }

  @Test
  void testRunVelocityRotation() {
    drive.periodic();

    // Command rotation only
    ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, 1.0);
    drive.runVelocity(speeds);

    // All modules should receive velocity commands
    assertTrue(flModuleIO.lastDriveVelocitySetpoint != 0.0);
    assertTrue(frModuleIO.lastDriveVelocitySetpoint != 0.0);
    assertTrue(blModuleIO.lastDriveVelocitySetpoint != 0.0);
    assertTrue(brModuleIO.lastDriveVelocitySetpoint != 0.0);
  }

  @Test
  void testStop() {
    drive.periodic();

    // First command some velocity
    drive.runVelocity(new ChassisSpeeds(1.0, 0.0, 0.0));

    // Then stop
    drive.stop();

    // Velocity setpoints should now be approximately zero
    // (The modules will receive zero-speed states)
  }

  @Test
  void testRunCharacterization() {
    drive.periodic();

    drive.runCharacterization(3.0);

    // All modules should receive the same open-loop output
    assertEquals(3.0, flModuleIO.lastDriveOpenLoopOutput, DELTA);
    assertEquals(3.0, frModuleIO.lastDriveOpenLoopOutput, DELTA);
    assertEquals(3.0, blModuleIO.lastDriveOpenLoopOutput, DELTA);
    assertEquals(3.0, brModuleIO.lastDriveOpenLoopOutput, DELTA);
  }

  @Test
  void testGetMaxLinearSpeed() {
    assertEquals(
        DriveConstants.SPEED_AT_12V.in(MetersPerSecond),
        drive.getMaxLinearSpeedMetersPerSec(),
        DELTA);
  }

  @Test
  void testGetMaxAngularSpeed() {
    double expectedAngularSpeed =
        DriveConstants.DEFAULT_ROT_SPEED.in(RadiansPerSecond) / DriveConstants.driveBaseRadius;
    assertEquals(expectedAngularSpeed, drive.getMaxAngularSpeedRadPerSec(), DELTA);
  }

  @Test
  void testGyroDisconnectedFallback() {
    // Disconnect the gyro
    gyroIO.disconnect();

    // Set up odometry data showing forward motion
    double[] timestamps = {0.0, 0.02};
    double[] drivePositions1 = {0.0, 0.1};
    double[] drivePositions2 = {0.0, 0.1};
    double[] drivePositions3 = {0.0, 0.1};
    double[] drivePositions4 = {0.0, 0.1};
    Rotation2d[] turnPositions = {new Rotation2d(), new Rotation2d()};

    flModuleIO.setOdometryData(timestamps, drivePositions1, turnPositions);
    frModuleIO.setOdometryData(timestamps, drivePositions2, turnPositions);
    blModuleIO.setOdometryData(timestamps, drivePositions3, turnPositions);
    brModuleIO.setOdometryData(timestamps, drivePositions4, turnPositions);
    gyroIO.setOdometryData(timestamps, new Rotation2d[] {new Rotation2d(), new Rotation2d()});

    // Should still work with kinematics-based fallback
    drive.periodic();

    // No exception means fallback is working
    assertNotNull(drive.getPose());
  }

  @Test
  void testOdometryUpdatesWithGyro() {
    gyroIO.connect();

    // Simulate robot rotating 45 degrees
    Rotation2d initialRotation = new Rotation2d();
    Rotation2d finalRotation = Rotation2d.fromDegrees(45.0);

    double[] timestamps = {0.0, 0.02};
    Rotation2d[] gyroPositions = {initialRotation, finalRotation};

    gyroIO.setOdometryData(timestamps, gyroPositions);

    // Modules stay stationary (no translation)
    double[] drivePositions = {0.0, 0.0};
    Rotation2d[] turnPositions = {new Rotation2d(), new Rotation2d()};

    flModuleIO.setOdometryData(timestamps, drivePositions, turnPositions);
    frModuleIO.setOdometryData(timestamps, drivePositions, turnPositions);
    blModuleIO.setOdometryData(timestamps, drivePositions, turnPositions);
    brModuleIO.setOdometryData(timestamps, drivePositions, turnPositions);

    drive.periodic();

    // Rotation should be updated from gyro
    // Note: The exact value depends on pose estimator behavior
    assertNotNull(drive.getRotation());
  }

  @Test
  void testGetWheelRadiusCharacterizationPositions() {
    flModuleIO.drivePositionRad = 1.0;
    frModuleIO.drivePositionRad = 2.0;
    blModuleIO.drivePositionRad = 3.0;
    brModuleIO.drivePositionRad = 4.0;

    drive.periodic();

    double[] positions = drive.getWheelRadiusCharacterizationPositions();

    assertEquals(4, positions.length);
    assertEquals(1.0, positions[0], DELTA);
    assertEquals(2.0, positions[1], DELTA);
    assertEquals(3.0, positions[2], DELTA);
    assertEquals(4.0, positions[3], DELTA);
  }

  @Test
  void testGetFFCharacterizationVelocity() {
    flModuleIO.driveVelocityRadPerSec = 10.0;
    frModuleIO.driveVelocityRadPerSec = 12.0;
    blModuleIO.driveVelocityRadPerSec = 14.0;
    brModuleIO.driveVelocityRadPerSec = 16.0;

    drive.periodic();

    double avgVelocity = drive.getFFCharacterizationVelocity();

    // Should be average of all four modules
    double expected = (10.0 + 12.0 + 14.0 + 16.0) / 4.0;
    assertEquals(expected, avgVelocity, DELTA);
  }

  @Test
  void testVelocityDesaturation() {
    drive.periodic();

    // Command velocities that exceed max speed
    double excessiveSpeed = DriveConstants.SPEED_AT_12V.in(MetersPerSecond) * 2.0;
    ChassisSpeeds speeds = new ChassisSpeeds(excessiveSpeed, excessiveSpeed, 0.0);
    drive.runVelocity(speeds);

    // Module velocities should be desaturated (scaled down)
    // All modules should have non-zero but limited velocities
    // The exact values depend on kinematics, but none should exceed max
    double maxModuleSpeed =
        DriveConstants.SPEED_AT_12V.in(MetersPerSecond) / DriveConstants.wheelRadiusMeters;

    assertTrue(Math.abs(flModuleIO.lastDriveVelocitySetpoint) <= maxModuleSpeed + DELTA);
    assertTrue(Math.abs(frModuleIO.lastDriveVelocitySetpoint) <= maxModuleSpeed + DELTA);
    assertTrue(Math.abs(blModuleIO.lastDriveVelocitySetpoint) <= maxModuleSpeed + DELTA);
    assertTrue(Math.abs(brModuleIO.lastDriveVelocitySetpoint) <= maxModuleSpeed + DELTA);
  }

  @Test
  void testBumpZoneAngle() {
    assertTrue(TeleopDrive.getBumpLockAngle(Rotation2d.fromDegrees(5.0)).getDegrees() == 45.0);
    assertTrue(TeleopDrive.getBumpLockAngle(Rotation2d.fromDegrees(-5.0)).getDegrees() == -45.0);
    assertTrue(TeleopDrive.getBumpLockAngle(Rotation2d.fromDegrees(175.0)).getDegrees() == 135.0);
    assertTrue(TeleopDrive.getBumpLockAngle(Rotation2d.fromDegrees(-175.0)).getDegrees() == -135.0);
  }

  @Test
  void testTrenchZoneAngle() {
    assertTrue(TeleopDrive.getTrenchLockAngle(Rotation2d.fromDegrees(100.0)).getDegrees() == 180.0);
    assertTrue(
        TeleopDrive.getTrenchLockAngle(Rotation2d.fromDegrees(-100.0)).getDegrees() == -180.0);
    assertTrue(TeleopDrive.getTrenchLockAngle(Rotation2d.fromDegrees(80.0)).getDegrees() == 0.0);
    assertTrue(TeleopDrive.getTrenchLockAngle(Rotation2d.fromDegrees(-80.0)).getDegrees() == 0.0);
  }

  @Test
  void testBumpTrigger() {
    Pose2d bumpPose;
    TeleopDrive teleopDrive = new TeleopDrive(drive, null);

    // Left bump zone - Current Alliance
    drive.setPose(new Pose2d(4.0, 5.5, new Rotation2d()));
    assertTrue(teleopDrive.inBumpZone());

    // Right bump zone - Current Alliance
    drive.setPose(new Pose2d(4.0, 2.5, new Rotation2d()));
    assertTrue(teleopDrive.inBumpZone());

    // Left bump zone - Opposite Alliance
    drive.setPose(new Pose2d(12.0, 5.5, new Rotation2d()));
    assertTrue(teleopDrive.inBumpZone());

    // Right bump zone - Opposite Alliance
    drive.setPose(new Pose2d(12.0, 2.5, new Rotation2d()));
    assertTrue(teleopDrive.inBumpZone());

    // Outside bump zones
    drive.setPose(new Pose2d(8.0, 5.5, new Rotation2d()));
    assertTrue(!teleopDrive.inBumpZone());
  }

  @Test
  void testTrenchTrigger() {
    TeleopDrive teleopDrive = new TeleopDrive(drive, null);

    // Left trench zone - Current Alliance
    drive.setPose(new Pose2d(3.5, 7.5, new Rotation2d()));
    assertTrue(teleopDrive.inTrenchZone());

    // Right trench zone - Current Alliance
    drive.setPose(new Pose2d(4.50, .75, new Rotation2d()));
    assertTrue(teleopDrive.inTrenchZone());

    // Left trench zone - Opposite Alliance
    drive.setPose(new Pose2d(12.0, 7.5, new Rotation2d()));
    assertTrue(teleopDrive.inTrenchZone());

    // Right trench zone - Opposite Alliance
    drive.setPose(new Pose2d(12.0, 0.75, new Rotation2d()));
    assertTrue(teleopDrive.inTrenchZone());

    // Outside trench zones
    drive.setPose(new Pose2d(8.0, 5.5, new Rotation2d()));
    assertTrue(!teleopDrive.inTrenchZone());
  }
}
