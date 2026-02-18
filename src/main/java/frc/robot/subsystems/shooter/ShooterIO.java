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

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean connected = false;

    public AngularVelocity velocityMotorLeft = RPM.of(0);
    public AngularVelocity velocityMotorRight = RPM.of(0);
    public AngularVelocity velocityShooterWheel = RPM.of(0);

    public Voltage appliedVoltageMotorLeft = Volts.of(0);
    public Voltage appliedVoltageMotorRight = Volts.of(0);

    public Current currentMotorLeft = Amps.of(0);
    public Current currentMotorRight = Amps.of(0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Run the motor at the specified open loop value. */
  public default void setOpenLoop(double output) {}

  /** Run the motor at the specified velocity in rad/sec. */
  public default void setVelocity(double velocityRadPerSec) {}

  /** Run the motor to the specified position in radians. */
  public default void setPosition(double positionRad) {}

  /** Stop the motor. */
  public default void stop() {}
}
