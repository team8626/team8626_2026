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

import edu.wpi.first.wpilibj.Timer;

/**
 * Simulation IO for the hopper subsystem. No real implementation may ever exist. This class is
 * purely for simulation purposes.
 */
public class HopperIOSim implements HopperIO {
  private int numFuel = 0;
  private Timer releaseTimer = new Timer();

  public HopperIOSim() {
    releaseTimer.reset();
    releaseTimer.start();
  }

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
  public boolean popFuel() {
    boolean retVal = false;
    if (numFuel > 0 && releaseTimer.hasElapsed(HopperConstants.POP_FUEL_TIME.in(Seconds))) {
      retVal = true;
      numFuel--;
      releaseTimer.reset();
      releaseTimer.start();
    }
    return retVal;
  }
}
