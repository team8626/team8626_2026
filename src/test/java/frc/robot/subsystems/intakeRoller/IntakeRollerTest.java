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

package frc.robot.subsystems.intakeRoller;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

@Tag("unit")
public class IntakeRollerTest {
  private static final double DELTA = 1e-3;

  private MockIntakeRollerIO io;
  private IntakeRoller intakeRoller;

  @BeforeAll
  static void initializeHAL() {
    HAL.initialize(500, 0);
  }

  @BeforeEach
  void setUp() {
    io = new MockIntakeRollerIO();
    intakeRoller = new IntakeRoller(io);
  }

  @Test
  void testRunVelocity() {
    intakeRoller.periodic();

    double targetVelocityRPM = 5.0;
    intakeRoller.runVelocity(RPM.of(targetVelocityRPM));

    assertEquals(targetVelocityRPM, io.lastVelocitySetpoint.in(RPM), DELTA);
  }

  @Test
  void testRunOpenLoop() {
    intakeRoller.periodic();

    double voltage = 6.0;
    intakeRoller.runOpenLoop(Volts.of(voltage));

    assertEquals(voltage, io.lastOpenLoopOutput.in(Volts), DELTA);
  }

  @Test
  void testStop() {
    intakeRoller.periodic();

    intakeRoller.runVelocity(RPM.of(100.0));
    intakeRoller.stop();

    assertTrue(io.stopCalled);
  }

  @Test
  void testGetVelocity() {
    io.velocity = RadiansPerSecond.of(8.0);
    intakeRoller.periodic();

    assertEquals(8.0, intakeRoller.getVelocity().in(RadiansPerSecond), DELTA);
  }

  @Test
  void testIsConnected() {
    io.connected = true;
    intakeRoller.periodic();
    assertTrue(intakeRoller.isConnected());

    io.disconnect();
    intakeRoller.periodic();
    assertFalse(intakeRoller.isConnected());
  }

  @Test
  void testGetAppliedVolts() {
    io.appliedVolts = Volts.of(3.5);
    intakeRoller.periodic();

    assertEquals(3.5, intakeRoller.getAppliedVoltage().in(Volts), DELTA);
  }

  @Test
  void testGetCurrentAmps() {
    io.currentAmps = Amps.of(15.0);
    intakeRoller.periodic();

    assertEquals(15.0, intakeRoller.getCurrent().in(Amps), DELTA);
  }

  @Test
  void testNegativeVelocity() {
    intakeRoller.periodic();

    intakeRoller.runVelocity(RPM.of(-200));

    assertEquals(-200, io.lastVelocitySetpoint.in(RPM), DELTA);
  }
}
