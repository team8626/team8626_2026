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
import static frc.robot.subsystems.intakeLinkage.IntakeLinkageConstants.*;
import static frc.robot.subsystems.intakeLinkage.IntakeLinkageConstants.GAINS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeLinkageIOSim implements IntakeLinkageIO {
  /**
   * Simulates intakeLinkage motor dynamics (Neo Vortex + gearbox) for physics-accurate behavior.
   */
  private final SingleJointedArmSim motorSim;
  /** PID used to correct position error when in position closed-loop mode. */
  private final PIDController positionController =
      new PIDController(GAINS.kP(), GAINS.kI(), GAINS.kD());

  private double appliedVolts = 0.0;
  private Angle desiredAngle = IntakeLinkageConstants.STARTING_ANGLE;

  public IntakeLinkageIOSim() {
    motorSim =
        new SingleJointedArmSim(
            DCMotor.getNeoVortex(1),
            IntakeLinkageConstants.GEAR_REDUCTION,
            IntakeLinkageConstants.ARM_INERTIA.in(KilogramSquareMeters),
            IntakeLinkageConstants.ARM_LENGTH.in(Meters),
            IntakeLinkageConstants.MIN_ANGLE.in(Radians),
            IntakeLinkageConstants.MAX_ANGLE.in(Radians),
            true,
            IntakeLinkageConstants.MIN_ANGLE.in(Radians),
            new double[0]);
  }

  @Override
  public void updateInputs(IntakeLinkageIOInputs inputs) {
    desiredAngle =
        Degrees.of(
            MathUtil.clamp(
                desiredAngle.in(Degrees),
                IntakeLinkageConstants.MIN_ANGLE.in(Degrees),
                IntakeLinkageConstants.MAX_ANGLE.in(Degrees)));

    motorSim.update(0.02);

    inputs.position = Degrees.of(motorSim.getAngleRads() * 180.0 / Math.PI);

    motorSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));

    inputs.connected = true;
    inputs.position = Degrees.of(motorSim.getAngleRads() * 180.0 / Math.PI);
    inputs.velocity = DegreesPerSecond.of(motorSim.getVelocityRadPerSec() * 180.0 / Math.PI);
    inputs.appliedVoltage = Volts.of(appliedVolts);
    inputs.current = Amps.of(Math.abs(motorSim.getCurrentDrawAmps()));
  }

  @Override
  public void setPosition(Angle position) {
    desiredAngle = position;
  }

  public void setPID(double new_kP, double new_kI, double new_kD) {
    positionController.setPID(new_kP, new_kI, new_kD);
  }

  @Override
  public void goUp(Angle offset) {
    desiredAngle.minus(offset);
  }

  @Override
  public void stow() {
    desiredAngle = IntakeLinkageConstants.STOW_ANGLE;
  }

  @Override
  public void deploy() {
    desiredAngle = IntakeLinkageConstants.DEPLOY_ANGLE;
  }

  @Override
  public void hopperOpen() {
    desiredAngle = IntakeLinkageConstants.HOPPER_OPEN_ANGLE;
  }

  @Override
  public void goDown(Angle offset) {
    desiredAngle.plus(offset);
  }
}
