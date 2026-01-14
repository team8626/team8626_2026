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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexIOInputsAutoLogged inputs = new IndexIOInputsAutoLogged();
  private final Alert motorDisconnectedAlert =
      new Alert("Index motor disconnected.", AlertType.kError);

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);

    // Update alerts
    motorDisconnectedAlert.set(!inputs.connected);
  }

  /**
   * Run the index at a specified velocity.
   *
   * @param velocityRadPerSec Velocity in radians per second
   */
  public void runVelocity(double velocityRadPerSec) {
    io.setVelocity(velocityRadPerSec);
  }

  /**
   * Run the index to a specified position.
   *
   * @param positionRad Position in radians
   */
  public void runToPosition(double positionRad) {
    io.setPosition(positionRad);
  }

  /**
   * Run the index to a specified angle.
   *
   * @param angle Rotation2d angle
   */
  public void runToAngle(Rotation2d angle) {
    io.setPosition(angle.getRadians());
  }

  /**
   * Advance the index to the next position (120 degrees forward). This method calculates the next
   * target position based on the current position and advances by one step (120 degrees).
   */
  public void advanceToNextPosition() {
    double currentPosition = inputs.positionRad;
    double targetPosition = currentPosition + positionStepRad;
    io.setPosition(targetPosition);
  }

  /**
   * Run the index at open loop voltage.
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

  /** Returns the current position in radians. */
  @AutoLogOutput
  public double getPositionRad() {
    return inputs.positionRad;
  }

  /** Returns the current position as a Rotation2d. */
  @AutoLogOutput
  public Rotation2d getAngle() {
    return new Rotation2d(inputs.positionRad);
  }

  /** Returns the current velocity in radians per second. */
  @AutoLogOutput
  public double getVelocityRadPerSec() {
    return inputs.velocityRadPerSec;
  }

  /** Returns whether the motor is connected. */
  public boolean isConnected() {
    return inputs.connected;
  }

  /** Returns the current applied voltage. */
  public double getAppliedVolts() {
    return inputs.appliedVolts;
  }

  /** Returns the current draw in amps. */
  public double getCurrentAmps() {
    return inputs.currentAmps;
  }
}
