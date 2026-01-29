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
 * Mock implementation of ModuleIO for unit testing. Provides direct control over all input values
 * without physics simulation.
 */
public class MockModuleIO implements ModuleIO {
  // Controllable input values
  public boolean driveConnected = true;
  public double drivePositionRad = 0.0;
  public double driveVelocityRadPerSec = 0.0;
  public double driveAppliedVolts = 0.0;
  public double driveCurrentAmps = 0.0;

  public boolean turnConnected = true;
  public Rotation2d turnPosition = new Rotation2d();
  public double turnVelocityRadPerSec = 0.0;
  public double turnAppliedVolts = 0.0;
  public double turnCurrentAmps = 0.0;

  public double[] odometryTimestamps = new double[] {0.0};
  public double[] odometryDrivePositionsRad = new double[] {0.0};
  public Rotation2d[] odometryTurnPositions = new Rotation2d[] {new Rotation2d()};

  // Capture values sent to the module
  public double lastDriveOpenLoopOutput = 0.0;
  public double lastTurnOpenLoopOutput = 0.0;
  public double lastDriveVelocitySetpoint = 0.0;
  public Rotation2d lastTurnPositionSetpoint = new Rotation2d();

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveConnected = driveConnected;
    inputs.drivePositionRad = drivePositionRad;
    inputs.driveVelocityRadPerSec = driveVelocityRadPerSec;
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = driveCurrentAmps;

    inputs.turnConnected = turnConnected;
    inputs.turnPosition = turnPosition;
    inputs.turnVelocityRadPerSec = turnVelocityRadPerSec;
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = turnCurrentAmps;

    inputs.odometryTimestamps = odometryTimestamps;
    inputs.odometryDrivePositionsRad = odometryDrivePositionsRad;
    inputs.odometryTurnPositions = odometryTurnPositions;
  }

  @Override
  public void setDriveOpenLoop(double output) {
    lastDriveOpenLoopOutput = output;
    driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    lastTurnOpenLoopOutput = output;
    turnAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    lastDriveVelocitySetpoint = velocityRadPerSec;
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    lastTurnPositionSetpoint = rotation;
  }

  /** Reset all values to defaults */
  public void reset() {
    driveConnected = true;
    drivePositionRad = 0.0;
    driveVelocityRadPerSec = 0.0;
    driveAppliedVolts = 0.0;
    driveCurrentAmps = 0.0;

    turnConnected = true;
    turnPosition = new Rotation2d();
    turnVelocityRadPerSec = 0.0;
    turnAppliedVolts = 0.0;
    turnCurrentAmps = 0.0;

    odometryTimestamps = new double[] {0.0};
    odometryDrivePositionsRad = new double[] {0.0};
    odometryTurnPositions = new Rotation2d[] {new Rotation2d()};

    lastDriveOpenLoopOutput = 0.0;
    lastTurnOpenLoopOutput = 0.0;
    lastDriveVelocitySetpoint = 0.0;
    lastTurnPositionSetpoint = new Rotation2d();
  }

  /**
   * Set odometry data for testing odometry processing.
   *
   * @param timestamps Array of timestamps
   * @param drivePositions Array of drive positions in radians
   * @param turnPositions Array of turn positions
   */
  public void setOdometryData(
      double[] timestamps, double[] drivePositions, Rotation2d[] turnPositions) {
    this.odometryTimestamps = timestamps;
    this.odometryDrivePositionsRad = drivePositions;
    this.odometryTurnPositions = turnPositions;
  }
}
