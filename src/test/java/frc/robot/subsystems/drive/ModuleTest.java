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

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

/** Unit tests for the Module class. */
@Tag("unit")
public class ModuleTest {
  private static final double DELTA = 1e-6;

  private MockModuleIO mockIO;
  private Module module;

  @BeforeEach
  void setUp() {
    // Initialize HAL for WPILib classes
    HAL.initialize(500, 0);

    mockIO = new MockModuleIO();
    module = new Module(mockIO, 0);
  }

  @Test
  void testGetAngle() {
    // Set a specific turn position
    mockIO.turnPosition = Rotation2d.fromDegrees(45.0);
    module.periodic();

    assertEquals(45.0, module.getAngle().getDegrees(), DELTA);
  }

  @Test
  void testGetPositionMeters() {
    // Set drive position in radians
    double drivePositionRad = 10.0; // 10 radians
    mockIO.drivePositionRad = drivePositionRad;
    mockIO.odometryDrivePositionsRad = new double[] {drivePositionRad};
    module.periodic();

    // Expected: position in radians * wheel radius
    double expectedMeters = drivePositionRad * DriveConstants.wheelRadiusMeters;
    assertEquals(expectedMeters, module.getPositionMeters(), DELTA);
  }

  @Test
  void testGetVelocityMetersPerSec() {
    // Set drive velocity in rad/sec
    double driveVelocityRadPerSec = 5.0;
    mockIO.driveVelocityRadPerSec = driveVelocityRadPerSec;
    module.periodic();

    // Expected: velocity in rad/sec * wheel radius
    double expectedVelocity = driveVelocityRadPerSec * DriveConstants.wheelRadiusMeters;
    assertEquals(expectedVelocity, module.getVelocityMetersPerSec(), DELTA);
  }

  @Test
  void testGetPosition() {
    // Set both drive position and turn angle
    mockIO.drivePositionRad = 8.0;
    mockIO.turnPosition = Rotation2d.fromDegrees(90.0);
    mockIO.odometryDrivePositionsRad = new double[] {8.0};
    mockIO.odometryTurnPositions = new Rotation2d[] {Rotation2d.fromDegrees(90.0)};
    module.periodic();

    SwerveModulePosition position = module.getPosition();

    assertEquals(8.0 * DriveConstants.wheelRadiusMeters, position.distanceMeters, DELTA);
    assertEquals(90.0, position.angle.getDegrees(), DELTA);
  }

  @Test
  void testGetState() {
    // Set velocity and angle
    mockIO.driveVelocityRadPerSec = 3.0;
    mockIO.turnPosition = Rotation2d.fromDegrees(180.0);
    module.periodic();

    SwerveModuleState state = module.getState();

    assertEquals(3.0 * DriveConstants.wheelRadiusMeters, state.speedMetersPerSecond, DELTA);
    assertEquals(180.0, state.angle.getDegrees(), DELTA);
  }

  @Test
  void testRunSetpoint() {
    // Set current module angle to match the setpoint so cosineScale doesn't reduce velocity
    mockIO.turnPosition = Rotation2d.fromDegrees(45.0);
    module.periodic();

    // Run a setpoint
    SwerveModuleState setpoint = new SwerveModuleState(2.0, Rotation2d.fromDegrees(45.0));
    module.runSetpoint(setpoint);

    // Verify velocity command was sent (converted from m/s to rad/s)
    double expectedVelocityRadPerSec =
        setpoint.speedMetersPerSecond / DriveConstants.wheelRadiusMeters;
    assertEquals(expectedVelocityRadPerSec, mockIO.lastDriveVelocitySetpoint, DELTA);

    // Verify turn position was set
    assertEquals(45.0, mockIO.lastTurnPositionSetpoint.getDegrees(), DELTA);
  }

  @Test
  void testRunSetpointOptimization() {
    // Module is at 0 degrees
    mockIO.turnPosition = Rotation2d.fromDegrees(0.0);
    module.periodic();

    // Request 180 degrees - should optimize to reverse direction instead
    SwerveModuleState setpoint = new SwerveModuleState(2.0, Rotation2d.fromDegrees(180.0));
    module.runSetpoint(setpoint);

    // After optimization, the state should have been modified
    // The module should turn to a closer angle and reverse velocity
    // Check that turn setpoint is closer to current position than 180 degrees
    double turnSetpointDegrees = mockIO.lastTurnPositionSetpoint.getDegrees();

    // The optimized angle should be 0 degrees (not 180) with reversed velocity
    assertTrue(
        Math.abs(turnSetpointDegrees) < 90.0 || Math.abs(turnSetpointDegrees - 360.0) < 90.0,
        "Turn setpoint should be optimized to a closer angle");
  }

  @Test
  void testRunCharacterization() {
    module.runCharacterization(6.0);

    // Should set open loop drive output
    assertEquals(6.0, mockIO.lastDriveOpenLoopOutput, DELTA);
    // Should set turn to zero degrees
    assertEquals(0.0, mockIO.lastTurnPositionSetpoint.getDegrees(), DELTA);
  }

  @Test
  void testStop() {
    // First set some non-zero values
    module.runCharacterization(6.0);

    // Now stop
    module.stop();

    // Both outputs should be zero
    assertEquals(0.0, mockIO.lastDriveOpenLoopOutput, DELTA);
    assertEquals(0.0, mockIO.lastTurnOpenLoopOutput, DELTA);
  }

  @Test
  void testGetOdometryPositions() {
    // Set odometry data with multiple samples
    double[] timestamps = {0.0, 0.02, 0.04};
    double[] drivePositions = {0.0, 1.0, 2.0};
    Rotation2d[] turnPositions = {
      Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(10.0), Rotation2d.fromDegrees(20.0)
    };

    mockIO.setOdometryData(timestamps, drivePositions, turnPositions);
    module.periodic();

    SwerveModulePosition[] positions = module.getOdometryPositions();

    assertEquals(3, positions.length);
    assertEquals(0.0 * DriveConstants.wheelRadiusMeters, positions[0].distanceMeters, DELTA);
    assertEquals(1.0 * DriveConstants.wheelRadiusMeters, positions[1].distanceMeters, DELTA);
    assertEquals(2.0 * DriveConstants.wheelRadiusMeters, positions[2].distanceMeters, DELTA);
    assertEquals(0.0, positions[0].angle.getDegrees(), DELTA);
    assertEquals(10.0, positions[1].angle.getDegrees(), DELTA);
    assertEquals(20.0, positions[2].angle.getDegrees(), DELTA);
  }

  @Test
  void testGetOdometryTimestamps() {
    double[] timestamps = {0.1, 0.12, 0.14};
    mockIO.odometryTimestamps = timestamps;
    mockIO.odometryDrivePositionsRad = new double[] {0.0, 0.0, 0.0};
    mockIO.odometryTurnPositions =
        new Rotation2d[] {new Rotation2d(), new Rotation2d(), new Rotation2d()};
    module.periodic();

    double[] returnedTimestamps = module.getOdometryTimestamps();

    assertArrayEquals(timestamps, returnedTimestamps, DELTA);
  }

  @Test
  void testGetWheelRadiusCharacterizationPosition() {
    mockIO.drivePositionRad = 15.5;
    module.periodic();

    assertEquals(15.5, module.getWheelRadiusCharacterizationPosition(), DELTA);
  }

  @Test
  void testGetFFCharacterizationVelocity() {
    mockIO.driveVelocityRadPerSec = 42.0;
    module.periodic();

    assertEquals(42.0, module.getFFCharacterizationVelocity(), DELTA);
  }

  @Test
  void testDisconnectedDriveMotor() {
    mockIO.driveConnected = false;
    module.periodic();

    // Module should still function, alerts are handled internally
    // Just verify no exceptions are thrown
    assertNotNull(module.getState());
  }

  @Test
  void testDisconnectedTurnMotor() {
    mockIO.turnConnected = false;
    module.periodic();

    // Module should still function, alerts are handled internally
    assertNotNull(module.getAngle());
  }
}
