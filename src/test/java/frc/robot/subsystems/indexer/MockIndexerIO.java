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
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * Mock IndexerIO for unit tests. Exposes inputs for injection and records outputs (setpoints,
 * stop).
 */
public class MockIndexerIO implements IndexerIO {
  public boolean connected = true;
  public Angle position = Radians.of(0.0);
  public AngularVelocity velocity = RadiansPerSecond.of(0.0);
  public Voltage appliedVolts = Volts.of(0.0);
  public Current currentAmps = Amps.of(0.0);

  public Voltage lastOpenLoopOutput = Volts.of(0.0);
  public AngularVelocity lastVelocitySetpoint = RPM.of(0.0);
  public boolean stopCalled = false;

  @Override
  public void updateInputs(IndexIOInputs inputs) {
    inputs.connected = connected;
    inputs.position = position;
    inputs.velocity = velocity;
    inputs.appliedVoltage = appliedVolts;
    inputs.current = currentAmps;
  }

  @Override
  public void setOpenLoop(Voltage output) {
    lastOpenLoopOutput = output;
    appliedVolts = output;
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    lastVelocitySetpoint = velocity;
  }

  @Override
  public void stop() {
    stopCalled = true;
    appliedVolts = Volts.of(0.0);
  }

  public void reset() {
    connected = true;
    position = Radians.of(0.0);
    velocity = RadiansPerSecond.of(0.0);
    appliedVolts = Volts.of(0.0);
    currentAmps = Amps.of(0.0);
    lastOpenLoopOutput = Volts.of(0.0);
    lastVelocitySetpoint = RPM.of(0.0);
    stopCalled = false;
  }

  public void disconnect() {
    connected = false;
  }
}
