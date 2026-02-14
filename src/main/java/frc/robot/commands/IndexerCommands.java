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

package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.Indexer;

public class IndexerCommands {

  private IndexerCommands() {}

  /** Default velocity */
  public static final AngularVelocity DEFAULT_VELOCITY = RPM.of(100);

  /**
   * Runs the indexer at the given velocity for the given duration, then stops.
   *
   * @param indexer The indexer subsystem
   * @param durationSeconds How long to run (seconds)
   * @param velocity Velocity (AngularVelocity) (positive = forward)
   * @return Command that runs for duration then stops
   */
  public static Command runForDuration(
      Indexer indexer, double durationSeconds, AngularVelocity velocity) {
    return Commands.run(() -> indexer.runVelocity(velocity), indexer)
        .withTimeout(durationSeconds)
        .finallyDo(indexer::stop);
  }

  /**
   * Runs the indexer at default velocity for the given duration, then stops.
   *
   * @param indexer The indexer subsystem
   * @param durationSeconds How long to run (seconds)
   * @return Command that runs for duration then stops
   */
  public static Command runForDuration(Indexer indexer, double durationSeconds) {
    return runForDuration(indexer, durationSeconds, DEFAULT_VELOCITY);
  }
}
