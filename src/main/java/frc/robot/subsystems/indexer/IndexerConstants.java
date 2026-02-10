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

public class IndexerConstants {
  // Hardware configuration
  public static final int indexCanId = 20; // TODO: Set actual CAN ID
  public static final boolean motorInverted = false; // TODO: Verify direction
  public static final int motorCurrentLimit = 40; // Amps

  // Mechanical configuration
  public static final double gearReduction = 10.0; // TODO: Set actual gear ratio (motor:mechanism)

  // Encoder conversion factors (motor rotations/RPM -> mechanism radians/rad/sec)
  public static final double encoderPositionFactor = (2.0 * Math.PI) / gearReduction;
  public static final double encoderVelocityFactor = (2.0 * Math.PI) / 60.0 / gearReduction;

  // Velocity control: PID + feedforward
  public static final double velocityKp = 0.1;
  public static final double velocityKd = 0.0;
  public static final double velocityKs = 0.05; // Static friction
  public static final double velocityKv = 0.12; // Approx 12V / 100 rad/s
}
