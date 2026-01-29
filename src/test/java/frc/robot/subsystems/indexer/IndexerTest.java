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

package frc.robot.subsystems.indexer;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

/** Unit tests for the Index subsystem. */
@Tag("unit")
public class IndexerTest {
  private static final double DELTA = 1e-3;

  private MockIndexerIO io;
  private Indexer indexer;

  @BeforeAll
  static void initializeHAL() {
    HAL.initialize(500, 0);
  }

  @BeforeEach
  void setUp() {
    io = new MockIndexerIO();
    indexer = new Indexer(io);
  }

  @Test
  void testInitialPosition() {
    indexer.periodic();
    assertEquals(0.0, indexer.getPositionRad(), DELTA);
    assertEquals(0.0, indexer.getAngle().getRadians(), DELTA);
  }

  @Test
  void testRunVelocity() {
    indexer.periodic();

    double targetVelocity = 5.0; // rad/sec
    indexer.runVelocity(targetVelocity);

    assertEquals(targetVelocity, io.lastVelocitySetpoint, DELTA);
  }

  @Test
  void testRunToPosition() {
    indexer.periodic();

    double targetPosition = Math.PI / 2; // 90 degrees
    indexer.runToPosition(targetPosition);

    assertEquals(targetPosition, io.lastPositionSetpoint, DELTA);
  }

  @Test
  void testRunToAngle() {
    indexer.periodic();

    Rotation2d targetAngle = Rotation2d.fromDegrees(45.0);
    indexer.runToAngle(targetAngle);

    assertEquals(targetAngle.getRadians(), io.lastPositionSetpoint, DELTA);
  }

  @Test
  void testAdvanceToNextPosition() {
    // Start at position 0
    io.positionRad = 0.0;
    indexer.periodic();

    // Advance to next position
    indexer.advanceToNextPosition();

    // Should advance by 120 degrees (2π/3 radians)
    assertEquals(2.0 * Math.PI / 3.0, io.lastPositionSetpoint, DELTA);
  }

  @Test
  void testAdvanceToNextPositionFromMiddle() {
    // Start at position 1 (120 degrees)
    io.positionRad = 2.0 * Math.PI / 3.0;
    indexer.periodic();

    // Advance to next position
    indexer.advanceToNextPosition();

    // Should advance to 240 degrees (4π/3 radians)
    assertEquals(4.0 * Math.PI / 3.0, io.lastPositionSetpoint, DELTA);
  }

  @Test
  void testAdvanceToNextPositionWrapsAround() {
    // Start at position 2 (240 degrees)
    io.positionRad = 4.0 * Math.PI / 3.0;
    indexer.periodic();

    // Advance to next position
    indexer.advanceToNextPosition();

    // Should advance to 360 degrees (2π radians), which wraps to position 0
    assertEquals(2.0 * Math.PI, io.lastPositionSetpoint, DELTA);
  }

  @Test
  void testRunOpenLoop() {
    indexer.periodic();

    double voltage = 6.0;
    indexer.runOpenLoop(voltage);

    assertEquals(voltage, io.lastOpenLoopOutput, DELTA);
  }

  @Test
  void testStop() {
    indexer.periodic();

    // Run motor first
    indexer.runVelocity(10.0);

    // Then stop
    indexer.stop();

    assertTrue(io.stopCalled);
  }

  @Test
  void testGetPositionRad() {
    io.positionRad = 1.5;
    indexer.periodic();

    assertEquals(1.5, indexer.getPositionRad(), DELTA);
  }

  @Test
  void testGetAngle() {
    io.positionRad = Math.PI / 4;
    indexer.periodic();

    assertEquals(Math.PI / 4, indexer.getAngle().getRadians(), DELTA);
    assertEquals(45.0, indexer.getAngle().getDegrees(), DELTA);
  }

  @Test
  void testGetVelocity() {
    io.velocityRadPerSec = 8.0;
    indexer.periodic();

    assertEquals(8.0, indexer.getVelocityRadPerSec(), DELTA);
  }

  @Test
  void testIsConnected() {
    io.connected = true;
    indexer.periodic();
    assertTrue(indexer.isConnected());

    io.disconnect();
    indexer.periodic();
    assertFalse(indexer.isConnected());
  }

  @Test
  void testGetAppliedVolts() {
    io.appliedVolts = 3.5;
    indexer.periodic();

    assertEquals(3.5, indexer.getAppliedVolts(), DELTA);
  }

  @Test
  void testGetCurrentAmps() {
    io.currentAmps = 15.0;
    indexer.periodic();

    assertEquals(15.0, indexer.getCurrentAmps(), DELTA);
  }

  @Test
  void testNegativeVelocity() {
    indexer.periodic();

    indexer.runVelocity(-3.0);

    assertEquals(-3.0, io.lastVelocitySetpoint, DELTA);
  }

  @Test
  void testNegativePosition() {
    indexer.periodic();

    indexer.runToPosition(-Math.PI / 4);

    assertEquals(-Math.PI / 4, io.lastPositionSetpoint, DELTA);
  }

  @Test
  void testPresetPositions() {
    indexer.periodic();

    // Test position 0
    indexer.runToPosition(IndexerConstants.position0Rad);
    assertEquals(IndexerConstants.position0Rad, io.lastPositionSetpoint, DELTA);

    // Test position 1 (120 degrees)
    indexer.runToPosition(IndexerConstants.position1Rad);
    assertEquals(IndexerConstants.position1Rad, io.lastPositionSetpoint, DELTA);

    // Test position 2 (240 degrees)
    indexer.runToPosition(IndexerConstants.position2Rad);
    assertEquals(IndexerConstants.position2Rad, io.lastPositionSetpoint, DELTA);
  }
}
