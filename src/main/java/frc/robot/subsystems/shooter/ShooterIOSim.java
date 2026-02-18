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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/** Physics sim implementation of shooter IO. */
public class ShooterIOSim implements ShooterIO {
  private final FlywheelSim wheelSim;

  private double kV = ShooterConstants.velocityKv;
  private double kS = ShooterConstants.velocityKs;

  private boolean velocityClosedLoop = false;

  private PIDController velocityController =
      new PIDController(ShooterConstants.velocityKp, 0, ShooterConstants.velocityKd);
  private double velocityFFVolts = 0.0;
  private double appliedVolts = 0.0;
  private AngularVelocity desiredWheelVelocity = RPM.of(0.0);

  public ShooterIOSim() {
    // Create motor sim model
    // Using NEO Vortex motor (same as drive motors) - adjust if using different motor
    // Moment of inertia estimate: 0.01 kg*m^2 (adjust based on your mechanism)

    wheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNeoVortex(2),
                ShooterConstants.flywheelMOI,
                ShooterConstants.gearReduction),
            DCMotor.getNeoVortex(2),
            0.00363458292);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Run closed-loop control
    if (velocityClosedLoop) {
      appliedVolts =
          velocityFFVolts + velocityController.calculate(wheelSim.getAngularVelocityRadPerSec());
    } else {
      velocityController.reset();
    }

    // Update simulation state
    wheelSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    wheelSim.update(0.02);

    // Update inputs
    inputs.connected = true;

    inputs.velocityMotorLeft = RadiansPerSecond.of(wheelSim.getAngularVelocityRadPerSec());
    inputs.velocityMotorRight = RadiansPerSecond.of(wheelSim.getAngularVelocityRadPerSec());
    inputs.velocityShooterWheel = RadiansPerSecond.of(wheelSim.getAngularVelocityRadPerSec() / ShooterConstants.gearReduction);
    inputs.desiredWheelVelocity = desiredWheelVelocity;

    inputs.appliedVoltageMotorLeft = Volts.of(appliedVolts);
    inputs.appliedVoltageMotorRight = Volts.of(appliedVolts);

    inputs.currentMotorLeft = Amps.of(Math.abs(wheelSim.getCurrentDrawAmps()));
    inputs.currentMotorRight = Amps.of(Math.abs(wheelSim.getCurrentDrawAmps()));
  }

  @Override
  public void setOpenLoop(double output) {
    velocityClosedLoop = false;
    appliedVolts = output;
  }

  @Override
  public void setVelocity(AngularVelocity newVelocity) {
    velocityClosedLoop = true;
    desiredWheelVelocity = newVelocity;

    AngularVelocity motorAngularVelocity =
        desiredWheelVelocity.times(ShooterConstants.gearReduction);

    velocityFFVolts =
        kS * Math.signum(motorAngularVelocity.in(RadiansPerSecond))
            + kV * motorAngularVelocity.in(RadiansPerSecond); // calculates FeedForwardVolts
    velocityController.setSetpoint(
        motorAngularVelocity.in(
            RadiansPerSecond)); // Sets closed loop goal (tells PID what to aim for)
  }

  @Override
  public void stop() {
    velocityClosedLoop = false;
    desiredWheelVelocity = RPM.zero();
    appliedVolts = 0.0;
  }

  @Override
  public void setPID(double new_kP, double new_kD, double new_kV, double new_kS) {
    velocityController.setPID(new_kP, 0, new_kD);
    kV = new_kV;
    kS = new_kS;
  }
}
