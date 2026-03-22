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

package frc.robot.subsystems.intakeRoller;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;

/**
 * Mock IntakeRollerIO for unit tests. Exposes inputs for injection and records outputs (setpoints,
 * stop).
 */
public class MockIntakeRollerIO implements IntakeRollerIO {
  public boolean connected = true;
  public Angle position = Radians.of(0.0);
  public AngularVelocity velocity = RadiansPerSecond.of(0.0);
  public Voltage appliedVolts = Volts.of(0.0);
  public Current currentAmps = Amps.of(0.0);

  public Voltage lastOpenLoopOutput = Volts.of(0.0);
  public AngularVelocity lastVelocitySetpoint = RPM.of(0.0);
  public boolean stopCalled = false;

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    inputs.connected = connected;
    inputs.velocityRPMRollers = velocity.in(RPM);
    inputs.appliedVoltage = appliedVolts;
    inputs.current = currentAmps;
  }

  @Override
  public void start(AngularVelocity velocity) {
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
