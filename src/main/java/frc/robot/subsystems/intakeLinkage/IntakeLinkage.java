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
import static frc.robot.subsystems.intakeLinkage.IntakeLinkageConstants.GAINS;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.LoggedTunableNumber;
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

  private final LoggedTunableNumber KP =
      new LoggedTunableNumber("IntakeLinkage/Gains/kP", GAINS.kP());
  private final LoggedTunableNumber KI =
      new LoggedTunableNumber("IntakeLinkage/Gains/kI", GAINS.kI());
  private final LoggedTunableNumber KD =
      new LoggedTunableNumber("IntakeLinkage/Gains/kD", GAINS.kD());
  private final LoggedTunableNumber KV =
      new LoggedTunableNumber("IntakeLinkage/Gains/kV", GAINS.kV());
  private final LoggedTunableNumber KS =
      new LoggedTunableNumber("IntakeLinkage/Gains/kS", GAINS.kS());
  private final LoggedTunableNumber KG =
      new LoggedTunableNumber("IntakeLinkage/Gains/kG", GAINS.kG());

  // Configure SysId
  private SysIdRoutine sysId =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(0.25).per(Second),
              Volts.of(1.0),
              Seconds.of(3.0),
              (state) -> Logger.recordOutput("Linkage/SysIdState", state.toString())),
          new SysIdRoutine.Mechanism((voltage) -> setVoltage(voltage.in(Volts)), null, this));

  public IntakeLinkage(IntakeLinkageIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeLinkage", inputs);
    updateTunables();

    // Show alert if the intake linkage motor is not connected.
    motorDisconnectedAlert.set(!inputs.connected);
  }

  @AutoLogOutput
  public Angle getPosition() {
    return Degrees.of(inputs.positionDeg);
  }

  public void setPosition(Angle position) {
    io.setPosition(position);
  }

  public boolean isConnected() {
    return inputs.connected;
  }

  public Voltage getAppliedVoltage() {
    return Volts.of(inputs.appliedVoltage);
  }

  public Current getCurrent() {
    return Amps.of(inputs.amps);
  }

  public void stow() {
    io.setPosition(IntakeLinkageConstants.STOW_ANGLE);
  }

  public void deploy() {
    io.setPosition(IntakeLinkageConstants.DEPLOY_ANGLE);
  }

  public void hopperOpen() {
    io.setPosition(IntakeLinkageConstants.HOPPER_OPEN_ANGLE);
  }

  private void updateTunables() {
    if (KP.hasChanged(hashCode())
        || KI.hasChanged(hashCode())
        || KD.hasChanged(hashCode())
        || KV.hasChanged(hashCode())
        || KG.hasChanged(hashCode())
        || KS.hasChanged(hashCode())) {
      io.setPID(KP.get(), KI.get(), KD.get(), KV.get(), KG.get(), KS.get());
    }
  }

  // Characterization methods
  public void setVoltage(double input) {
    io.setVoltage(input);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> setVoltage(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> setVoltage(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
  }
}
