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

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * Mock IndexerIO for unit tests. Exposes inputs for injection and records outputs (setpoints,
 * stop).
 */
public class MockIndexerIO implements IndexerIO {
  public boolean connected = true;
  public AngularVelocity actualWheelVelocity = RPM.of(0.0);
  public AngularVelocity desiredWheelVelocity = RPM.of(0.0);
  public Voltage appliedVoltage = Volts.of(0.0);
  public Current current = Amps.of(0.0);
  public boolean atGoal = true;

  public Voltage lastOpenLoopOutput = Volts.of(0.0);
  public AngularVelocity lastVelocitySetpoint = RPM.of(0.0);
  public boolean stopCalled = false;

  @Override
  public void updateInputs(IndexIOInputs inputs) {
    inputs.connected = connected;
    inputs.actualWheelVelocity = actualWheelVelocity;
    inputs.desiredWheelVelocity = desiredWheelVelocity;
    inputs.appliedVoltage = appliedVoltage;
    inputs.current = current;
    inputs.atGoal = atGoal;
  }

  @Override
  public void setOpenLoop(Voltage output) {
    lastOpenLoopOutput = output;
    appliedVoltage = output;
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    lastVelocitySetpoint = velocity;
  }

  @Override
  public void stop() {
    stopCalled = true;
    appliedVoltage = Volts.of(0.0);
  }

  public void reset() {
    connected = true;
    actualWheelVelocity = RPM.of(0.0);
    desiredWheelVelocity = RPM.of(0.0);
    appliedVoltage = Volts.of(0.0);
    current = Amps.of(0.0);
    atGoal = true;

    lastOpenLoopOutput = Volts.of(0.0);
    lastVelocitySetpoint = RPM.of(0.0);
    stopCalled = false;
  }

  public void disconnect() {
    connected = false;
  }
}
