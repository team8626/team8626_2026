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
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.indexer.IndexerConstants.gearReduction;
import static frc.robot.subsystems.indexer.IndexerConstants.velocityKd;
import static frc.robot.subsystems.indexer.IndexerConstants.velocityKp;
import static frc.robot.subsystems.indexer.IndexerConstants.velocityKs;
import static frc.robot.subsystems.indexer.IndexerConstants.velocityKv;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Simulation IO for the indexer subsystem.
 *
 * <p><b>Open-loop mode:</b> The motor is driven by a commanded voltage (or duty cycle) with no
 * feedback. Speed depends on load and battery voltage; useful for simple on/off or manual control
 * where exact speed is not critical.
 *
 * <p><b>Closed-loop (velocity) mode:</b> A target velocity (setpoint) is given, and the actual
 * velocity is measured and fed back. The controller adjusts the motor output so the actual velocity
 * tracks the setpoint despite load or voltage changes. This implementation uses a PID controller
 * plus feedforward (Ks, Kv) to achieve accurate, responsive velocity control.
 *
 * <p><b>PID controller:</b> Computes a correction from the velocity error (setpoint minus actual).
 *
 * <ul>
 *   <li><b>Kp</b> (proportional): Output proportional to error; larger Kp gives a stronger, faster
 *       response but can cause overshoot or oscillation.
 *   <li><b>Ki</b> (integral): Not used here (0). Would eliminate steady-state error over time.
 *   <li><b>Kd</b> (derivative): Responds to the rate of change of error; helps dampen overshoot and
 *       smooth the response.
 * </ul>
 *
 * Tuning (velocityKp, velocityKd in {@link IndexerConstants}) balances response speed and
 * stability.
 */
public class IndexerIOSim implements IndexerIO {
  /** Simulates indexer motor dynamics (Neo Vortex + gearbox) for physics-accurate behavior. */
  private final DCMotorSim motorSim;
  /** PID used to correct velocity error when in velocity closed-loop mode. */
  private final PIDController velocityController = new PIDController(velocityKp, 0, velocityKd);

  /** True when setVelocity() is active; false for open-loop (setOpenLoop/stop). */
  private boolean velocityClosedLoop = false;
  /** Feedforward voltage (Ks * sign + Kv * setpoint) added to PID output in velocity mode. */
  private double velocityFFVolts = 0.0;
  /**
   * Voltage commanded to the motor; written by open-loop or by velocity closed-loop in
   * updateInputs.
   */
  private double appliedVolts = 0.0;

  public IndexerIOSim() {
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.01, gearReduction),
            DCMotor.getNeoVortex(1));
  }

  @Override
  public void updateInputs(IndexIOInputs inputs) {
    if (velocityClosedLoop) {
      appliedVolts =
          velocityFFVolts + velocityController.calculate(motorSim.getAngularVelocityRadPerSec());
    } else {
      velocityController.reset();
    }

    motorSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    motorSim.update(0.02);

    inputs.connected = true;
    inputs.position = Radians.of(motorSim.getAngularPositionRad());
    inputs.velocity = RadiansPerSecond.of(motorSim.getAngularVelocityRadPerSec());
    inputs.appliedVoltage = Volts.of(appliedVolts);
    inputs.current = Amps.of(Math.abs(motorSim.getCurrentDrawAmps()));
  }

  @Override
  public void setOpenLoop(Voltage output) {
    velocityClosedLoop = false;
    appliedVolts = output.in(Volts);
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    velocityClosedLoop = true;
    velocityFFVolts =
        velocityKs * Math.signum(velocity.in(RadiansPerSecond))
            + velocityKv * velocity.in(RadiansPerSecond);
    velocityController.setSetpoint(velocity.in(RadiansPerSecond));
  }

  @Override
  public void stop() {
    velocityClosedLoop = false;
    appliedVolts = 0.0;
  }
}
