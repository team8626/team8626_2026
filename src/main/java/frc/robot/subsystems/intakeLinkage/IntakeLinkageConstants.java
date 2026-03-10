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

public class IntakeLinkageConstants {
  // Hardware configuration
  public static final int intakeLinkageCanId = 20; // TODO: Set actual CAN ID
  public static final boolean motorInverted = false; // TODO: Verify direction
  public static final int motorCurrentLimit = 40; // Amps

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

  public static final Distance armLength =
      Inches.of(14); // TODO: Set actual arm length (center of rotation to center of intake)
  public static final Mass armMass = Pounds.of(5.595);
  public static final MomentOfInertia armInertia = KilogramSquareMeters.of(0.16995);
  // Mechanical configuration
  public static final double gearReduction = 80.0 / 1.0;

  // Encoder conversion factors (motor rotations/RPM -> mechanism deg/rad/sec)
  public static final double encoderPositionFactor = 360.0 / gearReduction;
  public static final double encoderVelocityFactor = 360.0 / 60.0 / gearReduction;

  // Position control: PID + feedforward (used in closed-loop position mode)
  // positionKp — Proportional gain. Output = Kp * (setpoint - actual). Drives the motor harder when
  //   position error is large. Tune up if response is sluggish or never reaches setpoint; tune down
  //   if the intakeLinkage overshoots (goes past the target position then back), oscillates (keeps
  // moving back and forth around the target), or sounds rough.
  public static final double positionKp = 0.1;
  public static final double positionKi = 0.0;
  //
  // positionKd — Derivative gain. Responds to rate of change of error; dampens overshoot (going
  // past
  //   the target). Tune up if there is noticeable overshoot or oscillation after a step change
  //   (position shoots past then wobbles); tune down (or to 0) if the response becomes noisy or
  // twitchy
  //   from derivative kick or encoder noise (small bumps in the feedback make the output jump).

  public static final double positionKd = 0.0;
  //
  // positionKs — Feedforward: voltage to overcome static friction (same sign as velocity). Added so
  //   the motor can start moving at low speeds. Tune up if the intakeLinkage barely moves or stalls
  //   at
  //   low speed; tune down if it creeps when it should be stopped or feels too aggressive at low
  //   speed.
  public static final double positionKs = 0.05;
  //
  // positionKv — Feedforward: volts per (rad/s). Approximate linear relationship between velocity
  public static final double positionKv = 0.12;
}
