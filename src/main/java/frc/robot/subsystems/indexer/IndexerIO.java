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

package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexIOInputs {
    public boolean connected = false;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IndexIOInputs inputs) {}

  /** Run the motor at the specified open loop value. */
  public default void setOpenLoop(double output) {}

  /** Run the motor at the specified velocity in rad/sec. */
  public default void setVelocity(double velocityRadPerSec) {}

  /** Run the motor to the specified position in radians. */
  public default void setPosition(double positionRad) {}

  /** Stop the motor. */
  public default void stop() {}
}
