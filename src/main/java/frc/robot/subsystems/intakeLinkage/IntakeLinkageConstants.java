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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class IntakeLinkageConstants {
  // Hardware configuration
  public static final int intakeLinkageCanId = 20; // TODO: Set actual CAN ID
  public static final boolean motorInverted = false; // TODO: Verify direction
  public static final int motorCurrentLimit = 40; // Amps

  public static final Angle minAngle =
      Degrees.of(80); // TODO: Set actual min angle (fully retracted)
  public static final Angle maxAngle =
      Degrees.of(235); // TODO: Set actual max angle (fully extended)
  public static final Angle positionConversionFactor = Degrees.of(360);
  public static final AngularVelocity velocityConversionFactor =
      DegreesPerSecond.of(360 / 60.0); // degrees per second

  public static final int maxCurrent = 40; // Amps

  public static final Angle tolerance = Degrees.of(4);

  // Gear box ratio
  public static final double inputGear = 16; // TODO: Set actual input gear teeth count (motor gear)
  public static final double stageOneInput =
      56; // TODO: Set actual stage one input gear teeth count
  public static final double stageOneOutput =
      45; // ToDO: Set actual stage one output gear teeth count
  public static final double stageTwoInput =
      40; // TODO: Set actual stage two input gear teeth count
  public static final double stageTwoOutput =
      16; // TODO: Set actual stage two output gear teeth count
  public static final double outputGear =
      45; // todo: Set actual output gear teeth count (attached to arm)
  public static final double maxPlanetaryRatio = 20 / 1.0;

  public static final double gearBoxRatio =
      (stageOneInput / inputGear)
          * (stageTwoInput / stageOneOutput)
          * (outputGear / stageTwoOutput)
          * maxPlanetaryRatio;

  public static final Distance armLength =
      Inches.of(9.5); // TODO: Set actual arm length (center of rotation to center of intake)
  public static final Mass armMass =
      Pounds.of(10.0); // TODO: Set actual arm mass (including intake and any supported cargo)

  // Mechanical configuration
  public static final double gearReduction = 10.0; // TODO: Set actual gear ratio (motor:mechanism)

  // Encoder conversion factors (motor rotations/RPM -> mechanism radians/rad/sec)
  public static final double encoderPositionFactor = (2.0 * Math.PI) / gearReduction;
  public static final double encoderVelocityFactor = (2.0 * Math.PI) / 60.0 / gearReduction;

  // Velocity control: PID + feedforward (used in closed-loop velocity mode)
  //
  // velocityKp — Proportional gain. Output = Kp * (setpoint - actual). Drives the motor harder when
  //   velocity error is large. Tune up if response is sluggish or never reaches setpoint; tune down
  //   if the intakeLinkage overshoots (goes past the target speed then back), oscillates (keeps
  // speeding
  //   up and slowing down around the target), or sounds rough.
  public static final double positionKp = 0.1;
  //
  // velocityKd — Derivative gain. Responds to rate of change of error; dampens overshoot (going
  // past
  //   the target). Tune up if there is noticeable overshoot or oscillation after a step change
  //   (speed shoots past then wobbles); tune down (or to 0) if the response becomes noisy or
  // twitchy
  //   from derivative kick or encoder noise (small bumps in the feedback make the output jump).
  public static final double positionKd = 0.0;
  //
  // velocityKs — Feedforward: voltage to overcome static friction (same sign as velocity). Added so
  //   the motor can start moving at low speeds. Tune up if the intakeLinkage barely moves or stalls
  // at
  //   low speed; tune down if it creeps when it should be stopped or feels too aggressive at low
  //   speed.
  public static final double positionKs = 0.05;
  //
  // velocityKv — Feedforward: volts per (rad/s). Approximate linear relationship between velocity
  // and
  //   voltage (e.g. 12V / 100 rad/s ≈ 0.12). Tune up if the intakeLinkage runs slow for a given
  // setpoint;
  //   tune down if it runs too fast or the PID is fighting the feedforward (e.g. once settled, the
  //   controller keeps adding a big correction in the wrong direction).
  public static final double positionKv = 0.12;
}
