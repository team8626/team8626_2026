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

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.indexer.IndexerConstants.GAINS;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Indexer subsystem: runs a single motor at a set velocity (closed-loop) or open-loop voltage. Use
 * {@link #runVelocity(AngularVelocity)} for normal operation; use {@link #runOpenLoop(Voltage)} for
 * testing. Call {@link #stop()} to stop the motor.
 */
public class Indexer extends SubsystemBase {
  /** Hardware IO implementation (Spark or Simulated). */
  private final IndexerIO io;

  /** Cached inputs from IO, logged each period via AdvantageKit. */
  private final IndexIOInputsAutoLogged inputs = new IndexIOInputsAutoLogged();
  /** Shown on the dashboard when the index motor is not connected. */
  private final Alert motorDisconnectedAlert =
      new Alert("Index motor disconnected.", AlertType.kError);

  private final LoggedTunableNumber flywheelKP =
      new LoggedTunableNumber("Spindexer/Flywheel/kP", GAINS.kP());
  private final LoggedTunableNumber flywheelKI =
      new LoggedTunableNumber("Spindexer/Flywheel/kI", GAINS.kI());
  private final LoggedTunableNumber flywheelKD =
      new LoggedTunableNumber("Spindexer/Flywheel/kD", GAINS.kD());
  private final LoggedTunableNumber flywheelKV =
      new LoggedTunableNumber("Spindexer/Flywheel/kV", GAINS.kV());
  private final LoggedTunableNumber flywheelKS =
      new LoggedTunableNumber("Spindexer/Flywheel/kS", GAINS.kS());
  private final LoggedTunableNumber flywheelRPM =
      new LoggedTunableNumber(
          "Spindexer/Flywheel/WheelRPM", IndexerConstants.DEFAULT_VELOCITY.in(RPM));

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    updateTunables();
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
  public void runVelocity(AngularVelocity velocity) {
    AngularVelocity new_velocity = velocity;

    // Check if the velocity is in bounds before setting it.
    // otherwise set it to the max velocity with the same sign.
    // This prevents the controller from trying to reach an invalid setpoint.
    if ((velocity.abs(RPM)) > IndexerConstants.MAX_VELOCITY.in(RPM)) {
      new_velocity = RPM.of(IndexerConstants.MAX_VELOCITY.copySign(new_velocity, RPM));
    }
    io.setVelocity(new_velocity);
  }

  /**
   * Run the index motor at open-loop voltage (for testing).
   *
   * @param output Voltage output (-12.0 to 12.0)
   */
  public void runOpenLoop(Voltage output) {
    io.setOpenLoop(output);
  }

  /** Stop the index motor. */
  public void stop() {
    io.stop();
  }

  @AutoLogOutput
  public AngularVelocity getVelocity() {
    return inputs.currentVelocity;
  }

  public AngularVelocity getDesiredVelocity() {
    return inputs.desiredVelocity;
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

  private void updateTunables() {
    if (flywheelKP.hasChanged(hashCode())
        || flywheelKI.hasChanged(hashCode())
        || flywheelKD.hasChanged(hashCode())
        || flywheelKV.hasChanged(hashCode())
        || flywheelKS.hasChanged(hashCode())) {
      io.setPID(
          flywheelKP.get(), flywheelKI.get(), flywheelKD.get(), flywheelKV.get(), flywheelKS.get());
    }
  }

  public Command applyDashboardVelocity() {
    io.setVelocity(RPM.of(flywheelRPM.get()));
    return new InstantCommand();
  }
}
