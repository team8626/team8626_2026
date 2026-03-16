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
import frc.robot.Constants;

public class IntakeRollerConstants {

  // Tuned Values
  public static final AngularVelocity DEFAULT_VELOCITY = RPM.of(750);
  public static final AngularVelocity PLOW_VELOCITY = RPM.of(-1000);

  public static final Current MAX_CURRENT = Amps.of(50);

  public static final AngularVelocity VELOCITY_TOLERANCE = RPM.of(25);

  private static final double MOI = 1;

  // Flywheel Config
  public static final RollerConfig ROLLER_CONFIG =
      switch (Constants.robot) {
        case REBUILT_PHOENIX, REBUILT_AKIT -> new RollerConfig(
            1, false, Amps.of(50), 1.0 / 1.0, 2 * MOI);
        default -> new RollerConfig(1, false, Amps.of(50), 1.0 / 1.0, 2 * MOI);
      };

  // PID Constants
  public static final Gains GAINS =
      switch (Constants.robot) {
        case REBUILT_PHOENIX, REBUILT_AKIT -> new Gains(0.0001, 0.0, 0.0, 0.00015, 0.01, 0.0);
        default -> new Gains(0.05, 0.0, 0.0, 0.10395, 0.00296, 0.0);
      };

  public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}

  public record RollerConfig(
      int CANID, boolean INVERTED, Current MAX_CURRENT, double REDUCTION, double MOI) {}

  public static final AngularVelocity MAX_VELOCITY = RPM.of(6700 / ROLLER_CONFIG.REDUCTION());
}
