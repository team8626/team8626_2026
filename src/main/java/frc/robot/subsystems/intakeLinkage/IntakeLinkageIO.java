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

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
// import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeLinkageIO {
  @AutoLog
  public static class IntakeLinkageIOInputs {
    public boolean connected = false;
    public Angle position = Radians.of(0.0);
    public Angle desiredAngle = Radians.of(0.0);
    public Voltage appliedVoltage = Volts.of(0.0);
    public Current current = Amps.of(0.0);
    public Temperature temperature = Celsius.of(0.0);
    public AngularVelocity velocity = AngularVelocity.ofBaseUnits(0.0, RadiansPerSecond);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeLinkageIOInputs inputs) {}

  /** Run the motor at the specified angle position */
  public default void setPosition(Angle position) {}

  /** Stop the motor. */
  public default void stop() {}
}
