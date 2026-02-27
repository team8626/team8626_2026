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

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Intake Roller subsystem: runs a single motor at a set velocity (closed-loop) or open-loop
 * voltage. Use {@link #runVelocity(double)} for normal operation; use {@link #runOpenLoop(double)}
 * for testing. Call {@link #stop()} to stop the motor.
 */
public class IntakeRoller extends SubsystemBase {
  /** Hardware IO implementation (Spark or Simulated). */
  private final IntakeRollerIO io;

  /** Cached inputs from IO, logged each period via AdvantageKit. */
  private final IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();

  /** Shown on the dashboard when the intake roller motor is not connected. */
  private final Alert motorDisconnectedAlert =
      new Alert("Intake Roller motor disconnected.", AlertType.kError);

  public IntakeRoller(IntakeRollerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake Roller", inputs);

    // Show alert if the intake roller motor is not connected.
    motorDisconnectedAlert.set(!inputs.connected);
  }

  /**
   * Run the intake roller motor at a constant velocity.
   *
   * @param velocityRadPerSec Velocity in radians per second
   */
  public void runVelocity(AngularVelocity velocity) {
    io.setVelocity(velocity);
  }

  /**
   * Run the intake roller motor at open-loop voltage (for testing).
   *
   * @param output Voltage output (-12.0 to 12.0)
   */
  public void runOpenLoop(Voltage output) {
    io.setOpenLoop(output);
  }

  /** Stop the intake roller motor. */
  public void stop() {
    io.stop();
  }

  @AutoLogOutput
  public AngularVelocity getVelocity() {
    return inputs.velocity;
  }

  public boolean isConnected() {
    return inputs.connected;
  }

  public Voltage getAppliedVoltage() {
    return inputs.appliedVoltage;
  }

  public Current getCurrent() {
    return inputs.current;
  }
}
