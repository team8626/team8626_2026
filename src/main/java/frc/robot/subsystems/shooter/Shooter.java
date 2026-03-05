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
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final Alert motorDisconnectedAlert =
      new Alert("Shooter motor disconnected.", AlertType.kError);

  // Configure SysId
  private SysIdRoutine sysId =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              null,
              null,
              (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
          new SysIdRoutine.Mechanism(
              (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

  private final LoggedTunableNumber flywheelKP =
      new LoggedTunableNumber("Shooter/Flywheel/kP", ShooterConstants.SHOOTER_KP);
  private final LoggedTunableNumber flywheelKD =
      new LoggedTunableNumber("Shooter/Flywheel/kD", ShooterConstants.SHOOTER_KD);
  private final LoggedTunableNumber flywheelKV =
      new LoggedTunableNumber("Shooter/Flywheel/kV", ShooterConstants.SHOOTER_KV);
  private final LoggedTunableNumber flywheelKS =
      new LoggedTunableNumber("Shooter/Flywheel/kS", ShooterConstants.SHOOTER_KS);
  private final LoggedTunableNumber flywheelRPM =
      new LoggedTunableNumber(
          "Shooter/Flywheel/WheelRPM", ShooterConstants.DEFAULT_VELOCITY.in(RPM));

  private AngularVelocity desiredVelocity = ShooterConstants.DEFAULT_VELOCITY;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    updateTunables();
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
  public void setVelocity(AngularVelocity newVelocity) {
    // AngularVelocity velocity = newVelocity;
    // if ((velocity.abs(RPM)) > ShooterConstants.MAX_VELOCITY.in(RPM)) {
    //   newVelocity = RPM.of(ShooterConstants.MAX_VELOCITY.copySign(velocity, RPM));
    // }
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

  private void updateTunables() {
    if (flywheelKP.hasChanged(hashCode())
        || flywheelKD.hasChanged(hashCode())
        || flywheelKV.hasChanged(hashCode())
        || flywheelKS.hasChanged(hashCode())) {
      io.setPID(flywheelKP.get(), flywheelKD.get(), flywheelKV.get(), flywheelKS.get());
    }

    if (flywheelRPM.hasChanged(hashCode())) {
      desiredVelocity = (RPM.of(flywheelRPM.get()));
    }
  }

  public Command updateVelocityCommand() {
    io.setVelocity(desiredVelocity);
    return new Command() {};
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    io.setOpenLoop(output);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }
}
