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

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Mock implementation of GyroIO for unit testing. Provides direct control over all input values
 * without requiring hardware.
 */
public class MockGyroIO implements GyroIO {
  // Controllable input values
  public boolean connected = true;
  public Rotation2d yawPosition = new Rotation2d();
  public double yawVelocityRadPerSec = 0.0;
  public double[] odometryYawTimestamps = new double[] {0.0};
  public Rotation2d[] odometryYawPositions = new Rotation2d[] {new Rotation2d()};

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = connected;
    inputs.yawPosition = yawPosition;
    inputs.yawVelocityRadPerSec = yawVelocityRadPerSec;
    inputs.odometryYawTimestamps = odometryYawTimestamps;
    inputs.odometryYawPositions = odometryYawPositions;
  }

  /** Reset all values to defaults */
  public void reset() {
    connected = true;
    yawPosition = new Rotation2d();
    yawVelocityRadPerSec = 0.0;
    odometryYawTimestamps = new double[] {0.0};
    odometryYawPositions = new Rotation2d[] {new Rotation2d()};
  }

  /**
   * Set the yaw position.
   *
   * @param yaw The yaw rotation
   */
  public void setYaw(Rotation2d yaw) {
    this.yawPosition = yaw;
    this.odometryYawPositions = new Rotation2d[] {yaw};
  }

  /**
   * Set odometry data for testing odometry processing.
   *
   * @param timestamps Array of timestamps
   * @param yawPositions Array of yaw positions
   */
  public void setOdometryData(double[] timestamps, Rotation2d[] yawPositions) {
    this.odometryYawTimestamps = timestamps;
    this.odometryYawPositions = yawPositions;
  }

  /** Simulate a disconnected gyro */
  public void disconnect() {
    this.connected = false;
  }

  /** Simulate a connected gyro */
  public void connect() {
    this.connected = true;
  }
}
