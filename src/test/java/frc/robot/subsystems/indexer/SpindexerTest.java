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
public class SpindexerTest {
  private static final double DELTA = 1e-3;

  private IndexerIOSim io;
  private Indexer spindexer;

  @BeforeAll
  static void initializeHAL() {
    HAL.initialize(500, 0);
  }

  @BeforeEach
  void setUp() {
    io = new IndexerIOSim();
    spindexer = new Indexer(io);
  }

  // @Test
  // void testRunVelocity() throws InterruptedException {
  //   AngularVelocity targetVelocity = IndexerConstants.DEFAULT_VELOCITY;

  //   spindexer.runVelocity(targetVelocity);
  //   spindexer.periodic();

  //   System.out.printf(
  //       String.format(
  //           "[testRunVelocity] Using IndexerIOSim - Target: %.1f Rot/sec%n",
  //           targetVelocity.in(RotationsPerSecond)));

  //   assertTrue(targetVelocity.equals(spindexer.getVelocity()));

  //   // Simulate the scheduler (20ms per cycle) for 1 second to allow the controller to converge
  // to
  //   // the target velocity and measure time to converge to target velocity (assuming tolerance)
  //   int convergeTimeMs = 0;
  //   double errorRPS =
  //       Double.MAX_VALUE; // Rotations per second error between target and actual velocity

  //   for (int i = 1; i <= 50; i++) {
  //     spindexer.periodic();
  //     errorRPS =
  //         Math.abs(
  //             targetVelocity.in(RotationsPerSecond)
  //                 - spindexer.getVelocity().in(RotationsPerSecond));

  //     if (errorRPS < IndexerConstants.VELOCITY_TOLERANCE.in(RotationsPerSecond)) {
  //       convergeTimeMs = i * 20; // Time to converge in milliseconds
  //       System.out.printf(String.format("[testRunVelocity] Converged in %dms%n",
  // convergeTimeMs));
  //       break; // Converged to target velocity within tolerance
  //     }
  //     Thread.sleep(20); // Simulate 20ms scheduler cycle
  //   }

  //   assertTrue(
  //       errorRPS < IndexerConstants.VELOCITY_TOLERANCE.in(RotationsPerSecond),
  //       String.format(
  //           "[testRunVelocity] Velocity did not converge to target within tolerance. Final error:
  // %.1f Rot/sec",
  //           errorRPS));
  // }

  // @Test
  // void testStop() throws InterruptedException {
  //   AngularVelocity targetVelocity = IndexerConstants.DEFAULT_VELOCITY;

  //   spindexer.runVelocity(targetVelocity);
  //   spindexer.periodic();

  //   // Simulate the scheduler until speed is seen on the subsystem
  //   for (int i = 1; i <= 50; i++) {
  //     spindexer.periodic();
  //     if (spindexer.getVelocity().gt(RotationsPerSecond.of(0.0))) {
  //       break;
  //     }
  //   }

  //   assertTrue(
  //       spindexer.getVelocity().gt(RotationsPerSecond.of(0.0)),
  //       String.format("[testStop] Failed to start the Spindexer"));

  //   // Stop the Spindexer
  //   spindexer.stop();
  //   spindexer.periodic();

  //   assertTrue(
  //       spindexer.getVelocity().equals(RotationsPerSecond.of(0.0)),
  //       String.format("[testStop] The Spindexer did not stop"));
  // }

  // @Test
  // void testRunOpenLoop() {
  //   Voltage voltage = Volts.of(6);

  //   spindexer.runOpenLoop(voltage);
  //   spindexer.periodic();

  //   assertTrue(voltage.equals(spindexer.getAppliedVoltage()));
  // }

  @Test
  void testGetVelocity() {
    AngularVelocity targetVelocity = IndexerConstants.DEFAULT_VELOCITY;

    spindexer.start(targetVelocity);
    spindexer.periodic();

    // Simulate the scheduler until speed is seen on the subsystem
    for (int i = 1; i <= 50; i++) {
      spindexer.periodic();
      if (spindexer.getVelocity().gt(RotationsPerSecond.of(0.0))) {
        break;
      }
    }

    assertTrue(spindexer.getVelocity().gt(RotationsPerSecond.of(0.0)));
  }

  // @Test
  // void testIsConnected() {
  //   // Note: Simulation is connected by default
  //   spindexer.periodic();
  //   assertTrue(spindexer.isConnected());

  //   io.disconnect();
  //   spindexer.periodic();
  //   assertFalse(spindexer.isConnected());
  // }

  // @Test
  // void testGetAppliedVoltage() {
  //   Voltage voltage = Volts.of(6);

  //   spindexer.runOpenLoop(voltage);
  //   spindexer.periodic();

  //   assertTrue(voltage.equals(spindexer.getAppliedVoltage()));
  // }

  // @Test
  // void testGetCurrent() {
  //   Voltage voltage = Volts.of(6);

  //   spindexer.runOpenLoop(voltage);
  //   spindexer.periodic();

  //   assertNotEquals(spindexer.getCurrent().in(Amps), 0);
  // }

  //
  // TODO: verify tests and un-comment them
  //

  // @Test
  // void testNegativeVelocity() throws InterruptedException {
  //   AngularVelocity targetVelocity = IndexerConstants.DEFAULT_VELOCITY.negate();

  //   spindexer.runVelocity(targetVelocity);
  //   spindexer.periodic();

  //   System.out.printf(
  //       String.format(
  //           "[testRunVelocity] Using IndexerIOSim - Target: %.1f Rot/sec%n",
  //           targetVelocity.in(RotationsPerSecond)));

  //   assertTrue(targetVelocity.equals(spindexer.getVelocity()));

  //   // Simulate the scheduler (20ms per cycle) for 1 second to allow the controller to converge
  // to
  //   // the target velocity
  //   // and measure time to converge to target velocity (assuming tolerance)
  //   int convergeTimeMs = 0;
  //   double errorRPS =
  //       Double.MAX_VALUE; // Rotations per second error between target and actual velocity

  //   for (int i = 1; i <= 50; i++) {
  //     spindexer.periodic();
  //     errorRPS =
  //         Math.abs(
  //             targetVelocity.in(RotationsPerSecond)
  //                 - spindexer.getVelocity().in(RotationsPerSecond));

  //     if (errorRPS < IndexerConstants.VELOCITY_TOLERANCE.in(RotationsPerSecond)) {
  //       convergeTimeMs = i * 20; // Time to converge in milliseconds
  //       System.out.printf(
  //           String.format("[testNegativeVelocity] Converged in %dms%n", convergeTimeMs));
  //       break; // Converged to target velocity within tolerance
  //     }
  //     Thread.sleep(20); // Simulate 20ms scheduler cycle
  //   }

  //   assertTrue(
  //       errorRPS < IndexerConstants.VELOCITY_TOLERANCE.in(RotationsPerSecond),
  //       String.format(
  //           "[testNegativeVelocity] Velocity did not converge to target within tolerance. Final
  // error: %.1f Rot/sec",
  //           errorRPS));
  // }

  // @Test
  // void testMaxVelocity() {
  //   // Test Positive value out of bounds
  //   AngularVelocity targetVelocity = RPM.of(10000); // Exceeds max velocity
  //   spindexer.runVelocity(targetVelocity);
  //   spindexer.periodic();
  //   assertTrue(spindexer.getVelocity().equals(IndexerConstants.MAX_VELOCITY));

  //   // Test Negative value out of bounds
  //   targetVelocity = RPM.of(-10000); // Exceeds max velocity
  //   spindexer.runVelocity(targetVelocity);
  //   spindexer.periodic();
  //   assertEquals(
  //       spindexer.getVelocity().in(RPM),
  //       IndexerConstants.MAX_VELOCITY.copySign(targetVelocity, RPM),
  //       DELTA);
  // }
}
