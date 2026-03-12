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
import frc.robot.Constants;

public class IntakeLinkageConstants {
  // Hardware configuration
  public static final int INTAKE_LINKAGE_CAN_ID = 2;
  public static final boolean MOTOR_INVERTED = false; // TODO: Verify direction
  public static final Current MOTOR_CURRENT_LIMIT = Amps.of(40);

  public static final Angle MIN_ANGLE =
      Degrees.of(80); // TODO: Set actual min angle (fully retracted)
  public static final Angle MAX_ANGLE = Degrees.of(180);
  public static final Angle STARTING_ANGLE =
      Degrees.of(
          90); // TODO: Set actual starting angle (where the intake starts at the beginning of a
  // match)
  public static final Angle STOW_ANGLE =
      Degrees.of(100); // TODO: Set actual stow angle (where the intake should be when stowed
  public static final Angle DEPLOY_ANGLE =
      Degrees.of(160); // TODO: Set actual deploy angle (where the intake should be when deployed)
  public static final Angle HOPPER_OPEN_ANGLE =
      Degrees.of(145); // TODO: Set actual score angle (where the intake should be when scoring)
  public static final Angle POSITION_CONVERSION_FACTOR = Degrees.of(360);
  public static final AngularVelocity velocityConversionFactor =
      DegreesPerSecond.of(360 / 60.0); // degrees per second

  public static final int MAX_CURRENT = 40; // Amps

  public static final Angle TOLERANCE = Degrees.of(4);

  public static final Distance ARM_LENGTH =
      Inches.of(14); // TODO: Set actual arm length (center of rotation to center of intake)
  public static final Mass ARM_MASS = Pounds.of(5.595);
  public static final MomentOfInertia ARM_INERTIA = KilogramSquareMeters.of(0.16995);
  // Mechanical configuration
  public static final double GEAR_REDUCTION = 80.0 / 1.0;

  // Encoder conversion factors (motor rotations/RPM -> mechanism deg/rad/sec)
  public static final double ENCODER_POSITION_FACTOR = 360.0 / GEAR_REDUCTION;
  public static final double ENCODER_VELOCITY_FACTOR = 360.0 / 60.0 / GEAR_REDUCTION;

  public static final MotorConfig MOTOR_CONFIG =
      switch (Constants.robot) {
        case REBUILT_COMPBOT -> new MotorConfig(
            2, false, Amps.of(50), 3.0 / 1.0, 2 * ARM_INERTIA.in(KilogramSquareMeters));
        default -> new MotorConfig(
            0, false, Amps.of(50), 3.0 / 1.0, 2 * ARM_INERTIA.in(KilogramSquareMeters));
      };

  // PID Constants
  public static final Gains GAINS =
      switch (Constants.robot) {
        case REBUILT_COMPBOT -> new Gains(0.0001, 0.0, 0.0, 0.00015, 0.01, 0.0);
        default -> new Gains(0.05, 0.0, 0.0, 0.10395, 0.00296, 0.0);
      };

  public record Gains(double kP, double kI, double kD, double kS, double kG, double kV) {}

  public record MotorConfig(
      int CANID, boolean INVERTED, Current MAX_CURRENT, double REDUCTION, double MOI) {}
}
