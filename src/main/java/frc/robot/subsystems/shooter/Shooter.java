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

// import static frc.robot.subsystems.shooter.ShooterConstants.positionStepRad;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final Alert motorDisconnectedAlert =
      new Alert("Shooter motor disconnected.", AlertType.kError);

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    // Update alerts
    motorDisconnectedAlert.set(!inputs.connected);
  }

  /**
   * Run the shooter at a specified velocity.
   *
   * @param velocityRadPerSec Velocity in radians per second
   */
  public void runVelocity(double velocityRadPerSec) {
    io.setVelocity(velocityRadPerSec);
  }

  /**
   * Run the shooter to a specified position.
   *
   * @param positionRad Position in radians
   */
  public void runToPosition(double positionRad) {
    io.setPosition(positionRad);
  }

  /**
   * Run the shooter to a specified angle.
   *
   * @param angle Rotation2d angle
   */
  public void runToAngle(Rotation2d angle) {
    io.setPosition(angle.getRadians());
  }

  /**
   * Advance the shooter to the next position (120 degrees forward). This method calculates the next
   * target position based on the current position and advances by one step (120 degrees).
   */
  /*public void advanceToNextPosition() {
    double currentPosition = inputs.positionRad;//TODO: ask for removal
    double targetPosition = currentPosition + positionStepRad;
    io.setPosition(targetPosition);
  }*/

  /**
   * Run the shooter at open loop voltage.
   *
   * @param output Voltage output (-12.0 to 12.0)
   */
  public void runOpenLoop(double output) {
    io.setOpenLoop(output);
  }

  /** Stop the shooter motor. */
  public void stop() {
    io.stop();
  }

  /** Returns the current position in radians. */
  /*@AutoLogOutput
  public double getPositionRad() {
    return inputs.positionRad;
  }*/

  /** Returns the current position as a Rotation2d. */
  /*@AutoLogOutput
  public Rotation2d getAngle() {
    return new Rotation2d(inputs.positionRad);
  }*/

  /** Returns the current velocity in radians per second. */
  @AutoLogOutput
  public AngularVelocity getVelocityRadPerSec() {
    return inputs.velocityShooterWheel;
  }

  /** Returns whether the motor is connected. */
  public boolean isConnected() {
    return inputs.connected;
  }

  /** Returns the current applied voltage to the left motor(leader). */
  public Voltage getAppliedVoltsLeft() {
    return inputs.appliedVoltageMotorLeft;
  }

  /** Returns the current applied voltage to the right motor(follower). */
  public Voltage getAppliedVoltsRight() {
    return inputs.appliedVoltageMotorRight;
  }

  /** Returns the current applied voltage to both motors combined. */
  public Voltage getTotalAppliedVolts() {
    return inputs.appliedVoltageMotorLeft.plus(inputs.appliedVoltageMotorRight);
  }

  /** Returns the current draw in amps for the left motor(leader). */
  public Current getCurrentAmpsLeft() {
    return inputs.currentMotorLeft;
  }

  /** Returns the current draw in amps for the right motor(follower). */
  public Current getCurrentAmpsRight() {
    return inputs.currentMotorRight;
  }

  /** Returns the current draw in amps for both motors combined. */
  public Current getTotalCurrentAmps() {
    return inputs.currentMotorLeft.plus(inputs.currentMotorRight);
  }
}
