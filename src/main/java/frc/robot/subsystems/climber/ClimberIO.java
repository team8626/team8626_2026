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

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class IndexIOInputs {
    public Distance extension = Inches.of(0.0); // length of the rope
    public double numOfRotations = 0.0;
    public double hookPosition = 0.0; // 0 is the bottom
    public boolean connected = false;
    public Angle position = Radians.of(0.0);
    public AngularVelocity velocity = RadiansPerSecond.of(0.0);
    public Voltage appliedVoltage = Volts.of(0.0);
    public Current current = Amps.of(0.0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IndexIOInputs inputs) {}

  /** Run the motor at the specified open loop voltage. */
  public default void setOpenLoop(Voltage output) {}

  /** Run the motor at the specified velocity in rad/sec. */
  public default void setVelocity(AngularVelocity velocity) {}

  /** Stop the motor. */
  public default void stop() {}
}
