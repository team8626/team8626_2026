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

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.intakeRoller.IntakeRollerConstants.GAINS;

import edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * IntakeRoller subsystem: runs a single motor at a set velocity (closed-loop) or open-loop voltage.
 * Use {@link #start(AngularVelocity)} for normal operation; use {@link #runOpenLoop(Voltage)} for
 * testing. Call {@link #stop()} to stop the motor.
 */
public class IntakeRoller extends SubsystemBase {
  /** Hardware IO implementation (Spark or Simulated). */
  private final IntakeRollerIO io;

  /** Cached inputs from IO, logged each period via AdvantageKit. */
  private final IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();
  /** Shown on the dashboard when the IntakeRoller motor is not connected. */
  private final Alert motorDisconnectedAlert =
      new Alert("IntakeRoller motor disconnected.", AlertType.kError);

  private final LoggedTunableNumber rollerKP =
      new LoggedTunableNumber("IntakeRoller/Roller/kP", GAINS.kP());
  private final LoggedTunableNumber rollerKI =
      new LoggedTunableNumber("IntakeRoller/Roller/kI", GAINS.kI());
  private final LoggedTunableNumber rollerKD =
      new LoggedTunableNumber("IntakeRoller/Roller/kD", GAINS.kD());
  private final LoggedTunableNumber rollerKV =
      new LoggedTunableNumber("IntakeRoller/Roller/kV", GAINS.kV());
  private final LoggedTunableNumber rollerKS =
      new LoggedTunableNumber("IntakeRoller/Roller/kS", GAINS.kS());
  private final LoggedTunableNumber rollerRPM =
      new LoggedTunableNumber(
          "IntakeRoller/Roller/WheelRPM", IntakeRollerConstants.DEFAULT_VELOCITY.in(RPM));

  // Configure SysId
  private SysIdRoutine sysId =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              null,
              null,
              (state) -> Logger.recordOutput("Rollers/SysIdState", state.toString())),
          new SysIdRoutine.Mechanism((voltage) -> setVoltage(voltage.in(Volts)), null, this));

  public IntakeRoller(IntakeRollerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    updateTunables();
    io.updateInputs(inputs);
    Logger.processInputs("IntakeRoller", inputs);

    // Show alert if the IntakeRoller motor is not connected.
    motorDisconnectedAlert.set(!inputs.connected);
  }

  /**
   * Run the IntakeRoller motor at a constant velocity.
   *
   * @param new_velocity Angular Velocity
   */
  public void start(AngularVelocity new_velocity) {
    AngularVelocity velocity = new_velocity;

    if ((new_velocity.abs(RPM)) > IntakeRollerConstants.MAX_VELOCITY.in(RPM)) {
      velocity = RPM.of(IntakeRollerConstants.MAX_VELOCITY.copySign(velocity, RPM));
    }
    io.setVelocity(velocity);
  }

  public void start() {
    start(RPM.of(rollerRPM.get()));
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
    return RPM.of(inputs.velocityRPMRollers);
  }

  public AngularVelocity getDesiredVelocity() {
    return RPM.of(inputs.velocityRPMDesired);
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
    if (rollerKP.hasChanged(hashCode())
        || rollerKI.hasChanged(hashCode())
        || rollerKD.hasChanged(hashCode())
        || rollerKV.hasChanged(hashCode())
        || rollerKS.hasChanged(hashCode())) {
      io.setPID(rollerKP.get(), rollerKI.get(), rollerKD.get(), rollerKV.get(), rollerKS.get());
    }
  }

  // Characterization methods
  public void setVoltage(double input) {
    io.setVoltage(input);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> setVoltage(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> setVoltage(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }
}
