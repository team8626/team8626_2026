// Copyright 2026 FRC 8626
// http://github.com/team8626
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

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.Indexer;

public class IndexerCommands {

  private IndexerCommands() {}

  /**
   * Creates a command that runs the index mechanism at a constant velocity for 8 seconds, then
   * stops.
   *
   * @param indexer The index subsystem
   * @param velocityRadPerSec The velocity to run at in radians per second
   * @return A command that runs the index for 8 seconds
   */
  public static Command runFor8Seconds(Indexer indexer, double velocityRadPerSec) {
    return Commands.run(() -> indexer.runVelocity(velocityRadPerSec), indexer)
        .withTimeout(8.0)
        .finallyDo(() -> indexer.stop());
  }

  /**
   * Creates a command that runs the index mechanism at a constant velocity for 8 seconds, then
   * stops. Uses a default velocity suitable for indexing game pieces.
   *
   * @param indexer The index subsystem
   * @return A command that runs the index for 8 seconds at default velocity
   */
  public static Command runFor8Seconds(Indexer indexer) {
    // Default velocity: one full rotation (2π radians) per second
    return runFor8Seconds(indexer, 2.0 * Math.PI);
  }

  /**
   * Test command that runs the index at a constant open-loop voltage for 8 seconds. Use this to
   * verify the motor is connected and responding before tuning PID.
   *
   * @param indexer The index subsystem
   * @return A command that runs the index at open loop for 8 seconds
   */
  public static Command testOpenLoop(Indexer indexer) {
    return Commands.run(() -> indexer.runOpenLoop(3.0), indexer)
        .withTimeout(8.0)
        .finallyDo(() -> indexer.stop());
  }
}
