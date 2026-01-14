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

import static frc.robot.subsystems.indexer.IndexerConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Physics sim implementation of index IO. */
public class IndexerIOSim implements IndexerIO {
  private final DCMotorSim motorSim;

  private boolean velocityClosedLoop = false;
  private boolean positionClosedLoop = false;
  private PIDController velocityController = new PIDController(velocityKp, 0, velocityKd);
  private PIDController positionController = new PIDController(positionKp, 0, positionKd);
  private double velocityFFVolts = 0.0;
  private double appliedVolts = 0.0;

  public IndexerIOSim() {
    // Create motor sim model
    // Using NEO Vortex motor (same as drive motors) - adjust if using different motor
    // Moment of inertia estimate: 0.01 kg*m^2 (adjust based on your mechanism)
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.01, gearReduction),
            DCMotor.getNeoVortex(1));
  }

  @Override
  public void updateInputs(IndexIOInputs inputs) {
    // Run closed-loop control
    if (velocityClosedLoop) {
      appliedVolts =
          velocityFFVolts + velocityController.calculate(motorSim.getAngularVelocityRadPerSec());
    } else if (positionClosedLoop) {
      appliedVolts = positionController.calculate(motorSim.getAngularPositionRad());
    } else {
      velocityController.reset();
      positionController.reset();
    }

    // Update simulation state
    motorSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    motorSim.update(0.02);

    // Update inputs
    inputs.connected = true;
    inputs.positionRad = motorSim.getAngularPositionRad();
    inputs.velocityRadPerSec = motorSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(motorSim.getCurrentDrawAmps());
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
    velocityFFVolts = velocityKs * Math.signum(velocityRadPerSec) + velocityKv * velocityRadPerSec;
    velocityController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setPosition(double positionRad) {
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
