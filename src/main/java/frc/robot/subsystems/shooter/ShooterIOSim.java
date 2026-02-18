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

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/** Physics sim implementation of shooter IO. */
public class ShooterIOSim implements ShooterIO {
  private final FlywheelSim wheelSim;

  private boolean velocityClosedLoop = false;

  @SuppressWarnings(
      "unused") // TODO: remove once positionClosedLoop is removed, just here to suppress warnings
  private boolean positionClosedLoop = false; // TODO: Ask for removal

  private PIDController velocityController = new PIDController(velocityKp, 0, velocityKd);
  private PIDController positionController = new PIDController(positionKp, 0, positionKd);
  private double velocityFFVolts = 0.0;
  private double appliedVolts = 0.0;

  public ShooterIOSim() {
    // Create motor sim model
    // Using NEO Vortex motor (same as drive motors) - adjust if using different motor
    // Moment of inertia estimate: 0.01 kg*m^2 (adjust based on your mechanism)

    wheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNeoVortex(2), flywheelMOI, gearReduction),
            DCMotor.getNeoVortex(2),
            0.00363458292);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Run closed-loop control
    if (velocityClosedLoop) {
      appliedVolts =
          velocityFFVolts + velocityController.calculate(wheelSim.getAngularVelocityRadPerSec());
    } /*else if (positionClosedLoop) {
        appliedVolts =
            positionController.calculate(
                wheelSim.getAngularPositionRad()); // TODO: ask Mr. Dumet if this is needed
      }*/ else {
      velocityController.reset();
      positionController.reset();
    }

    // Update simulation state
    wheelSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    wheelSim.update(0.02);

    // Update inputs
    inputs.connected = true;

    inputs.velocityMotorLeft = RadiansPerSecond.of(wheelSim.getAngularVelocityRadPerSec());
    inputs.velocityMotorRight = RadiansPerSecond.of(wheelSim.getAngularVelocityRadPerSec());
    inputs.velocityShooterWheel = RadiansPerSecond.of(0); // TODO: get the correct value

    inputs.appliedVoltageMotorLeft = Volts.of(appliedVolts);
    inputs.appliedVoltageMotorRight = Volts.of(appliedVolts);

    inputs.currentMotorLeft = Amps.of(Math.abs(wheelSim.getCurrentDrawAmps()));
    inputs.currentMotorRight = Amps.of(Math.abs(wheelSim.getCurrentDrawAmps()));
  }

  @Override
  public void setOpenLoop(double output) {
    velocityClosedLoop = false;
    positionClosedLoop = false;
    appliedVolts = output;
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    velocityClosedLoop = true;
    positionClosedLoop = false;
    velocityFFVolts =
        velocityKs * Math.signum(velocityRadPerSec)
            + velocityKv * velocityRadPerSec; // calculates FeedForwardVolts
    velocityController.setSetpoint(
        velocityRadPerSec); // Sets closed loop goal (tells PID what to aim for)
  }

  @Override
  public void setPosition(double positionRad) { // TODO: Ask Mr. Dumet for removal
    velocityClosedLoop = false;
    positionClosedLoop = true;
    positionController.setSetpoint(positionRad);
  }

  @Override
  public void stop() {
    velocityClosedLoop = false;
    positionClosedLoop = false;
    appliedVolts = 0.0;
  }
}
