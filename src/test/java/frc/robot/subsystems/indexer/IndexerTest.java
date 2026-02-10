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
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

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

    double targetVelocity = 5.0;
    indexer.runVelocity(targetVelocity);

    assertEquals(targetVelocity, io.lastVelocitySetpoint, DELTA);
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

    indexer.runVelocity(10.0);
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
}
