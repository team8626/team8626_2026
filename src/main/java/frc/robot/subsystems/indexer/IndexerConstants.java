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

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;

public class IndexerConstants {
  // Default values
  public static final AngularVelocity DEFAULT_VELOCITY = RPM.of(1500.0);
  public static final AngularVelocity VELOCITY_TOLERANCE = RPM.of(100.0);

  // Mechanical configuration
  public static final double GEAR_REDUCTION = 3.0;
  public static final AngularVelocity MAX_VELOCITY =
      RPM.of(5676.0 / GEAR_REDUCTION); // Neo Vortex free speed at output shaft

  // Hardware configuration
  public static final int indexCanId = 20; // TODO: Set actual CAN ID
  public static final boolean motorInverted = false; // TODO: Verify direction
  public static final int motorCurrentLimit = 40; // Amps

  // Encoder conversion factors (motor rotations/RPM -> mechanism radians/rad/sec)
  public static final double encoderPositionFactor = (2.0 * Math.PI) / GEAR_REDUCTION;
  public static final double encoderVelocityFactor = (2.0 * Math.PI) / 60.0 / GEAR_REDUCTION;

  // Velocity control: PID + feedforward (used in closed-loop velocity mode)
  //
  // velocityKp — Proportional gain. Output = Kp * (setpoint - actual). Drives the motor harder when
  //   velocity error is large. Tune up if response is sluggish or never reaches setpoint; tune down
  //   if the indexer overshoots (goes past the target speed then back), oscillates (keeps speeding
  //   up and slowing down around the target), or sounds rough.
  public static final double velocityKp = 0.38;
  //
  // velocityKd — Derivative gain. Responds to rate of change of error; dampens overshoot (going
  // past
  //   the target). Tune up if there is noticeable overshoot or oscillation after a step change
  //   (speed shoots past then wobbles); tune down (or to 0) if the response becomes noisy or
  // twitchy
  //   from derivative kick or encoder noise (small bumps in the feedback make the output jump).
  public static final double velocityKd = 0.0;
  //
  // velocityKs — Feedforward: voltage to overcome static friction (same sign as velocity). Added so
  //   the motor can start moving at low speeds. Tune up if the indexer barely moves or stalls at
  //   low speed; tune down if it creeps when it should be stopped or feels too aggressive at low
  //   speed.
  public static final double velocityKs = 0.5;
  //
  // velocityKv — Feedforward: volts per (rad/s). Approximate linear relationship between velocity
  // and
  //   voltage (e.g. 12V / 100 rad/s ≈ 0.12). Tune up if the indexer runs slow for a given setpoint;
  //   tune down if it runs too fast or the PID is fighting the feedforward (e.g. once settled, the
  //   controller keeps adding a big correction in the wrong direction).
  public static final double velocityKv = 0.012;
}
