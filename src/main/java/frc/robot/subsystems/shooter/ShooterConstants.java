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

public class ShooterConstants {
  // Hardware configuration
  public static final int shooterLeadCanId = 20; // TODO: Set actual CAN ID
  public static final int shooterFollowCanId = 21; // TODO: Set actual CAN ID
  public static final boolean motorInverted = false; // TODO: Verify direction
  public static final int motorCurrentLimit = 40; // Amps

  // Mechanical configuration
  public static final double gearReduction = 1;
  public static final double flywheelMOI = 19.4;
  // Encoder conversion factors
  // Convert motor rotations to mechanism radians
  public static final double encoderPositionFactor =
      (2.0 * Math.PI) / gearReduction; // Motor rotations -> Mechanism radians
  public static final double encoderVelocityFactor =
      (2.0 * Math.PI) / 60.0 / gearReduction; // Motor RPM -> Mechanism rad/sec

  // Velocity control PID (slot 0)
  public static final double velocityKp = 0.1; // Start conservative, increase if response is slow
  public static final double velocityKd = 0.0;
  public static final double velocityKs = 0.05; // Small voltage to overcome static friction
  public static final double velocityKv = 0.12; // Rough estimate: 12V / 100 rad/s = 0.12

  // Position control PID (slot 1)
  public static final double positionKp = 1.0; // TODO: Tune - start with 1.0 and adjust
  public static final double positionKd = 0.0;

  // Preset positions for 3-position shooting (120° apart)
  public static final double position0Rad = 0.0; // Position 0
  public static final double position1Rad = 2.0 * Math.PI / 3.0; // 120 degrees
  public static final double position2Rad = 4.0 * Math.PI / 3.0; // 240 degrees
  public static final double positionStepRad = 2.0 * Math.PI / 3.0; // Step size (120 degrees)
}
