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

import static edu.wpi.first.units.Units.*;

/**
 * Simulation IO for the hopper subsystem. No real implementation may ever exist. This class is
 * purely for simulation purposes.
 */
public class HopperIOSim implements HopperIO {
  private int numFuel = 0;

  public HopperIOSim() {}

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.numFuel = numFuel;
    inputs.ableToIntake = ableToIntake();
  }

  @Override
  public boolean ableToIntake() {
    return numFuel < HopperConstants.MAX_FUEL;
  }

  @Override
  public boolean isFull() {
    return !ableToIntake();
  }

  @Override
  public void pushFuel() {
    numFuel++;
  }

  @Override
  public void popFuel() {
    numFuel--;
  }
}
