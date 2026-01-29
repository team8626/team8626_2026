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
import edu.wpi.first.wpilibj.Timer;

/** Simulated gyro IO for integration testing. Integrates angular velocity to track heading. */
public class GyroIOSim implements GyroIO {
  private double yawPositionRad = 0.0;
  private double yawVelocityRadPerSec = 0.0;
  private double lastUpdateTime = Timer.getFPGATimestamp();

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    // Integrate velocity to get position
    double currentTime = Timer.getFPGATimestamp();
    double dt = currentTime - lastUpdateTime;
    lastUpdateTime = currentTime;

    yawPositionRad += yawVelocityRadPerSec * dt;

    // Normalize to -pi to pi
    while (yawPositionRad > Math.PI) yawPositionRad -= 2.0 * Math.PI;
    while (yawPositionRad < -Math.PI) yawPositionRad += 2.0 * Math.PI;

    inputs.connected = true;
    inputs.yawPosition = new Rotation2d(yawPositionRad);
    inputs.yawVelocityRadPerSec = yawVelocityRadPerSec;
    inputs.odometryYawTimestamps = new double[] {currentTime};
    inputs.odometryYawPositions = new Rotation2d[] {inputs.yawPosition};
  }

  /** Set the angular velocity for simulation. */
  public void setAngularVelocity(double velocityRadPerSec) {
    this.yawVelocityRadPerSec = velocityRadPerSec;
  }

  /** Set the yaw position directly. */
  public void setYaw(Rotation2d yaw) {
    this.yawPositionRad = yaw.getRadians();
  }

  /** Reset the gyro to zero. */
  public void reset() {
    this.yawPositionRad = 0.0;
    this.yawVelocityRadPerSec = 0.0;
    this.lastUpdateTime = Timer.getFPGATimestamp();
  }
}
