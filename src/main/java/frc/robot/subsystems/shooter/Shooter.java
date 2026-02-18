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

import static edu.wpi.first.units.Units.RPM;

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
  public void runVelocity(AngularVelocity velocity) {
    AngularVelocity newVelocity = velocity;

    if ((velocity.abs(RPM)) > ShooterConstants.MAX_VELOCITY.in(RPM)) {
      newVelocity = RPM.of(ShooterConstants.MAX_VELOCITY.copySign(velocity, RPM));
    }
    io.setVelocity(newVelocity);
  }

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

  /** Returns the current velocity in radians per second. */
  @AutoLogOutput
  public AngularVelocity getVelocity() {
    return inputs.velocityShooterWheel;
  }

  public boolean isAtGoal() {
    return inputs.isAtGoal;
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
