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

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Indexer subsystem: runs a single motor at a set velocity (closed-loop) or open-loop voltage. Use
 * {@link #runVelocity(double)} for normal operation; use {@link #runOpenLoop(double)} for testing.
 * Call {@link #stop()} to stop the motor.
 */
public class Indexer extends SubsystemBase {
  /** Hardware IO implementation (Spark or Simulated). */
  private final IndexerIO io;

  /** Cached inputs from IO, logged each period via AdvantageKit. */
  private final IndexIOInputsAutoLogged inputs = new IndexIOInputsAutoLogged();

  /** Shown on the dashboard when the index motor is not connected. */
  private final Alert motorDisconnectedAlert =
      new Alert("Index motor disconnected.", AlertType.kError);

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);

    // Show alert if the index motor is not connected.
    motorDisconnectedAlert.set(!inputs.connected);
  }

  /**
   * Run the index motor at a constant velocity.
   *
   * @param velocityRadPerSec Velocity in radians per second
   */
  public void runVelocity(double velocityRadPerSec) {
    io.setVelocity(velocityRadPerSec);
  }

  /**
   * Run the index motor at open-loop voltage (for testing).
   *
   * @param output Voltage output (-12.0 to 12.0)
   */
  public void runOpenLoop(double output) {
    io.setOpenLoop(output);
  }

  /** Stop the index motor. */
  public void stop() {
    io.stop();
  }

  @AutoLogOutput
  public double getPositionRad() {
    return inputs.positionRad;
  }

  @AutoLogOutput
  public Rotation2d getAngle() {
    return new Rotation2d(inputs.positionRad);
  }

  @AutoLogOutput
  public double getVelocityRadPerSec() {
    return inputs.velocityRadPerSec;
  }

  public boolean isConnected() {
    return inputs.connected;
  }

  public double getAppliedVolts() {
    return inputs.appliedVolts;
  }

  public double getCurrentAmps() {
    return inputs.currentAmps;
  }
}
