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
import org.littletonrobotics.junction.AutoLog;

public interface IntakeLinkageIO {
  @AutoLog
  public static class IntakeLinkageIOInputs {
    public boolean connected = false;
    public double positionDeg = 0;
    public double desiredDeg = IntakeLinkageConstants.MIN_ANGLE.in(Degree);
    public double appliedVoltage = 0;
    public double amps = 0;
    public double tempCelsius = 0;
    public boolean atGoal = false;
    public boolean isEnabled = false;
    public double velocityDegPerSec = 0;

    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
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

  public default void setPID(double kP, double kI, double kD, double kV, double kG, double kS) {}

  public default void setVoltage(double input) {}
}
