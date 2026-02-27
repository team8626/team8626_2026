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

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;

public class IntakeRollerConstants {
  // Hardware configuration
  public static final int intakeRollerCanId = 20; // TODO: Set actual CAN ID
  public static final boolean motorInverted = false; // TODO: Verify direction
  public static final int motorCurrentLimit = 40; // Amps

  // Mechanical configuration
  public static final double gearReduction = 1.0;
  public static final AngularVelocity intakeRollerIntakeSpeed =
      RPM.of(3500.0); // TODO: Set actual RPM

  // Encoder conversion factors (motor rotations/RPM -> mechanism radians/rad/sec)
  public static final double encoderPositionFactor = (2.0 * Math.PI) / gearReduction;
  public static final double encoderVelocityFactor = (2.0 * Math.PI) / 60.0 / gearReduction;

  // Velocity control: PID + feedforward (used in closed-loop velocity mode)
  //
  // velocityKp — Proportional gain. Output = Kp * (setpoint - actual). Drives the motor harder when
  //   velocity error is large. Tune up if response is sluggish or never reaches setpoint; tune down
  //   if the indexer overshoots (goes past the target speed then back), oscillates (keeps speeding
  //   up and slowing down around the target), or sounds rough.
  public static final double velocityKp = 0.1;
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
  public static final double velocityKs = 0.05;
  //
  // velocityKv — Feedforward: volts per (rad/s). Approximate linear relationship between velocity
  // and
  //   voltage (e.g. 12V / 100 rad/s ≈ 0.12). Tune up if the indexer runs slow for a given setpoint;
  //   tune down if it runs too fast or the PID is fighting the feedforward (e.g. once settled, the
  //   controller keeps adding a big correction in the wrong direction).
  public static final double velocityKv = 0.12;
}
