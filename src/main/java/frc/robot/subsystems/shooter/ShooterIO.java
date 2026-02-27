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
    public boolean isAtGoal = true;

    public AngularVelocity velocityMotorLeft = RPM.of(0);
    public AngularVelocity velocityMotorRight = RPM.of(0);
    public AngularVelocity velocityShooterWheel = RPM.of(0);
    public AngularVelocity desiredWheelVelocity = RPM.zero();

    public Voltage appliedVoltageMotorLeft = Volts.of(0);
    public Voltage appliedVoltageMotorRight = Volts.of(0);

    public Current currentMotorLeft = Amps.of(0);
    public Current currentMotorRight = Amps.of(0);
  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(ShooterIOInputs inputs);

  /** Run the motor at the specified open loop value. */
  public void setOpenLoop(double output);

  /** Run the motor at the specified velocity in rad/sec. */
  public void setVelocity(AngularVelocity velocity);

  /** Stop the motor. */
  public void stop();

  /** set the PID */
  public void setPID(double new_kP, double new_kD, double new_kV, double new_kS);
}
