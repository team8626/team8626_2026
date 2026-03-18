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

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollerIO {
  @AutoLog
  public static class IntakeRollerIOInputs {
    public boolean connected = false;
    public boolean atGoal = true;
    public boolean isEnabled = false;

    public double velocityRPMRollers = 0;
    public double velocityRPMMotor = 0;
    public double velocityRPMDesired = 0;

    public Voltage appliedVoltage = Volts.of(0.0);
    public Current current = Amps.of(0.0);
    public Temperature temperature = Celsius.of(0.0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeRollerIOInputs inputs) {}

  /** Run the motor at the specified open loop voltage. */
  public default void setOpenLoop(Voltage output) {}

  /** Run the motor at the specified --WHEEL-- velocity */
  public default void setVelocity(AngularVelocity velocity) {}

  /** Stop the motor. */
  public default void stop() {}

  /** Set the PID constants for the motor controller. */
  public default void setPID(double kP, double kI, double kD, double kV, double kS) {}

  public default void setVoltage(double input) {}
}
