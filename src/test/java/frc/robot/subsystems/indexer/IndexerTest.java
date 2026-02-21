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

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.AngularVelocity;
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
  void testRunVelocity() {
    indexer.periodic();

    AngularVelocity targetVelocityRPM = RPM.of(500);
    indexer.runVelocity(targetVelocityRPM);

    assertEquals(targetVelocityRPM.in(RPM), io.lastVelocitySetpoint.in(RPM), DELTA);
  }

  @Test
  void testRunOpenLoop() {
    indexer.periodic();

    double voltage = 6.0;
    indexer.runOpenLoop(Volts.of(voltage));

    assertEquals(voltage, io.lastOpenLoopOutput.in(Volts), DELTA);
  }

  @Test
  void testStop() {
    indexer.periodic();

    indexer.runVelocity(RPM.of(100.0));
    indexer.stop();

    assertTrue(io.stopCalled);
  }

  @Test
  void testGetVelocity() {
    AngularVelocity velocityRPM = RPM.of(500);
    io.actualWheelVelocity = velocityRPM;
    indexer.periodic();

    assertEquals(velocityRPM.in(RPM), indexer.getVelocity().in(RPM), DELTA);
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
    io.appliedVoltage = Volts.of(3.5);
    indexer.periodic();

    assertEquals(3.5, indexer.getAppliedVoltage().in(Volts), DELTA);
  }

  @Test
  void testGetCurrentAmps() {
    io.current = Amps.of(15.0);
    indexer.periodic();

    assertEquals(15.0, indexer.getCurrent().in(Amps), DELTA);
  }

  @Test
  void testNegativeVelocity() {
    AngularVelocity velocityRPM = RPM.of(-500);
    io.actualWheelVelocity = velocityRPM;

    indexer.periodic();

    indexer.runVelocity(velocityRPM);

    assertEquals(velocityRPM.in(RPM), io.lastVelocitySetpoint.in(RPM), DELTA);
  }

  @Test
  void testMaxVelocity() {
    // Test Positive value out of bounds
    AngularVelocity velocity = RPM.of(10000); // Exceeds max velocity
    io.desiredWheelVelocity = velocity;

    indexer.periodic();
    indexer.runVelocity(velocity);

    assertEquals(IndexerConstants.MAX_VELOCITY.in(RPM), io.lastVelocitySetpoint.in(RPM), DELTA);

    // Test Negative value out of bounds
    velocity = RPM.of(-10000); // Exceeds max velocity
    io.desiredWheelVelocity = velocity;

    indexer.periodic();
    indexer.runVelocity(velocity);

    assertEquals(
        IndexerConstants.MAX_VELOCITY.copySign(velocity, RPM),
        io.lastVelocitySetpoint.in(RPM),
        DELTA);
  }
}
