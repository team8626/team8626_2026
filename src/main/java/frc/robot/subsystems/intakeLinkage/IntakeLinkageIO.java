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

import edu.wpi.first.units.measure.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeLinkageIO {
  @AutoLog
  public static class IntakeLinkageIOInputs {
    public boolean connected = false;
    public Angle position = Degrees.of(0.0);
    public Angle desiredAngle = IntakeLinkageConstants.MIN_ANGLE;
    public Voltage appliedVoltage = Volts.of(0.0);
    public Current current = Amps.of(0.0);
    public Temperature temperature = Celsius.of(0.0);
    public AngularVelocity velocity = AngularVelocity.ofBaseUnits(0.0, DegreesPerSecond);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeLinkageIOInputs inputs) {}

  /** Run the motor at the specified angle position */
  public default void setPosition(Angle position) {}

  public default void goUp(Angle offset) {}

  public default void goDown(Angle offset) {}

  public default void stow() {}

  public default void deploy() {}

  public default void hopperOpen() {}

  public default void setPID(double kP, double kI, double kD) {}
}
