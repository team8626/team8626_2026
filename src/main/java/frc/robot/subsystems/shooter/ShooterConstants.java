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

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

public class ShooterConstants {
  // Hardware configuration
  public static final int LEFT_CANID = 7;
  public static final int RIGHT_CANID = 6;
  public static final boolean SHOOTER_LEFT_INVERTED = false;
  public static final Current CURRENT_LIMIT = Amps.of(40);

  // Mechanical configuration
  public static final double GEAR_REDUCTION = 1;
  public static final AngularVelocity MAX_VELOCITY = RPM.of(5676.0 / GEAR_REDUCTION);
  public static final AngularVelocity DEFAULT_VELOCITY =
      RPM.of(1500); // This is Flywheel RPM (not motor RPM)

  public static final double flywheelMOI = 0.00568;

  public static final Distance FLYWHEEL_RADIUS = Inches.of(2.0);
  public static final Angle SHOOTER_ANGLE = Degrees.of(55);
  public static final Transform3d SHOOTER_OFFSET =
      new Transform3d(
          Inches.of(-8),
          Inches.of(4.5),
          Inches.of(16.25),
          new Rotation3d(new Rotation2d(Degrees.of(0))));

  // Encoder conversion factors
  // Convert motor rotations to mechanism radians
  //   public static final double ENCODER_POSITION_FACTOR =
  //       (2.0 * Math.PI) / GEAR_REDUCTION; // Motor rotations -> Mechanism radians
  //   public static final double encoderVelocityFactor =
  //       (2.0 * Math.PI) / 60.0 / GEAR_REDUCTION; // Motor RPM -> Mechanism rad/sec

  // Velocity control PID (slot 0)
  public static final double SHOOTER_KP = 0.456; // Start conservative, increase if response is slow
  public static final double SHOOTER_KI = 0.0;
  public static final double SHOOTER_KD = 0.0;

  public static final double SHOOTER_KS = 0.05; // Small voltage to overcome static friction
  public static final double SHOOTER_KV = 0.12; // Rough estimate: 12V / 100 rad/s = 0.12
}
