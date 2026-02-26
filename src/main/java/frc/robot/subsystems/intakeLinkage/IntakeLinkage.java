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

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** IntakeLinkage subsystem: runs a single motor at a set velocity (closed-loop). */
public class IntakeLinkage extends SubsystemBase {
  /** Hardware IO implementation (Spark or Simulated). */
  private final IntakeLinkageIO io;

  /** Cached inputs from IO, logged each period via AdvantageKit. */
  private final IntakeLinkageIOInputsAutoLogged inputs = new IntakeLinkageIOInputsAutoLogged();

  /** Shown on the dashboard when the intakeLinkage motor is not connected. */
  private final Alert motorDisconnectedAlert =
      new Alert("IntakeLinkage motor disconnected.", AlertType.kError);

  public IntakeLinkage(IntakeLinkageIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeLinkage", inputs);

    // Show alert if the intake linkage motor is not connected.
    motorDisconnectedAlert.set(!inputs.connected);
  }

  /** Stop the intake linkage motor. */
  public void stop() {
    io.stop();
  }

  @AutoLogOutput
  public Angle getPosition() {
    return inputs.position;
  }

  @AutoLogOutput
  public Rotation2d getAngle() {
    return new Rotation2d(inputs.position.in(Radians));
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
