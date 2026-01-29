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

/**
 * Mock implementation of IndexIO for unit testing. Provides direct control over all input values
 * without hardware.
 */
public class MockIndexerIO implements IndexerIO {
  // Controllable input values
  public boolean connected = true;
  public double positionRad = 0.0;
  public double velocityRadPerSec = 0.0;
  public double appliedVolts = 0.0;
  public double currentAmps = 0.0;

  // Capture values sent to the motor
  public double lastOpenLoopOutput = 0.0;
  public double lastVelocitySetpoint = 0.0;
  public double lastPositionSetpoint = 0.0;
  public boolean stopCalled = false;

  @Override
  public void updateInputs(IndexIOInputs inputs) {
    inputs.connected = connected;
    inputs.positionRad = positionRad;
    inputs.velocityRadPerSec = velocityRadPerSec;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = currentAmps;
  }

  @Override
  public void setOpenLoop(double output) {
    lastOpenLoopOutput = output;
    appliedVolts = output;
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    lastVelocitySetpoint = velocityRadPerSec;
  }

  @Override
  public void setPosition(double positionRad) {
    lastPositionSetpoint = positionRad;
  }

  @Override
  public void stop() {
    stopCalled = true;
    appliedVolts = 0.0;
  }

  /** Reset all values to defaults */
  public void reset() {
    connected = true;
    positionRad = 0.0;
    velocityRadPerSec = 0.0;
    appliedVolts = 0.0;
    currentAmps = 0.0;
    lastOpenLoopOutput = 0.0;
    lastVelocitySetpoint = 0.0;
    lastPositionSetpoint = 0.0;
    stopCalled = false;
  }

  /** Disconnect the motor for testing error conditions */
  public void disconnect() {
    connected = false;
  }
}
