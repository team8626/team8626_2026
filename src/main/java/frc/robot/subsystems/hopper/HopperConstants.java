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

package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.units.measure.Time;

public class HopperConstants {
  public static final int MAX_FUEL = 30;
  public static Time POP_FUEL_TIME = Milliseconds.of(200); // 5 Fuel per second
}
