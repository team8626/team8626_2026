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

package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
  @AutoLog
  public static class HopperIOInputs {
    public boolean ableToIntake = true;
    public int numFuel = 0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(HopperIOInputs inputs) {}

  public abstract boolean ableToIntake();

  public abstract boolean isFull();

  public default void pushFuel() {}

  public default boolean popFuel() {
    return false;
  }
}
