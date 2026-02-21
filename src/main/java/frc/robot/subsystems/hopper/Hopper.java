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

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Indexer subsystem: runs a single motor at a set velocity (closed-loop) or open-loop voltage. Use
 * {@link #runVelocity(AngularVelocity)} for normal operation; use {@link #runOpenLoop(Voltage)} for
 * testing. Call {@link #stop()} to stop the motor.
 */
public class Hopper extends SubsystemBase {
  /** Hardware IO implementation (Spark or Simulated). */
  private final HopperIO io;

  /** Cached inputs from IO, logged each period via AdvantageKit. */
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  public Hopper(HopperIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);
  }

  public boolean ableToIntake() {
    return inputs.ableToIntake;
  }

  public void pushFuel() {
    io.pushFuel();
  }

  public void popFuel() {
    io.popFuel();
  }
}
