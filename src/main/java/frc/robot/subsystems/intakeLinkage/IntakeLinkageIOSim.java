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

package frc.robot.subsystems.intakeLinkage;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intakeLinkage.IntakeLinkageConstants.gearReduction;
import static frc.robot.subsystems.intakeLinkage.IntakeLinkageConstants.positionKd;
import static frc.robot.subsystems.intakeLinkage.IntakeLinkageConstants.positionKp;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Simulation IO for the IntakeLinkage subsystem.
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
 * Tuning (velocityKp, velocityKd in {@link IntakeLinkageConstants}) balances response speed and
 * stability.
 */
public class IntakeLinkageIOSim implements IntakeLinkageIO {
  /**
   * Simulates intakeLinkage motor dynamics (Neo Vortex + gearbox) for physics-accurate behavior.
   */
  private final DCMotorSim motorSim;
  /** PID used to correct position error when in position closed-loop mode. */
  private final PIDController positionController = new PIDController(positionKp, 0, positionKd);

  /** True when setPosition() is active; false for open-loop (setOpenLoop/stop). */
  private boolean positionClosedLoop = false;

  private double appliedVolts = 0.0;
  private double positionGoalRad;

  public IntakeLinkageIOSim() {
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.01, gearReduction),
            DCMotor.getNeoVortex(1));
  }

  @Override
  public void updateInputs(IntakeLinkageIOInputs inputs) {

    if (positionClosedLoop) {

      double currentRad = motorSim.getAngularPositionRad();

      appliedVolts = positionController.calculate(currentRad, positionGoalRad);

    } else {

      positionController.reset();
    }

    motorSim.update(0.02);

    inputs.position = Radians.of(motorSim.getAngularPositionRad());

    motorSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));

    inputs.connected = true;
    inputs.position = Radians.of(motorSim.getAngularPositionRad());
    inputs.velocity = RadiansPerSecond.of(motorSim.getAngularVelocityRadPerSec());
    inputs.appliedVoltage = Volts.of(appliedVolts);
    inputs.current = Amps.of(Math.abs(motorSim.getCurrentDrawAmps()));
  }

  @Override
  public void setPosition(Angle position) {
    positionGoalRad = position.in(Radians);
    positionClosedLoop = true;
  }

  @Override
  public void stop() {
    positionClosedLoop = false;
    appliedVolts = 0.0;
  }
}
