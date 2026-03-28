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

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;

public class ClimberConstants {

  // Inputs Values
  public static final Voltage CLIMB_VOLTAGE = Volts.of(-8);
  public static final Voltage STOW_VOLTAGE = Volts.of(-4);
  public static final Voltage STOW_SLOW_VOLTAGE = Volts.of(-0.5);
  public static final Voltage EXTEND_VOLTAGE = Volts.of(4);
  public static final Voltage ZERO_VOLTAGE = Volts.of(-1);
  public static final Voltage CLIMB_LOCK_VOLTAGE = Volts.of(-0.5);

  public static final Current STALL_CURRENT = Amps.of(10);
  public static final AngularVelocity STALL_ANGULAR_VELOCITY = RadiansPerSecond.of(6);

  public static final Angle CLIMB_POSITION = Rotations.of(75);
  public static final Angle AUTO_CLIMB_POSITION = Rotations.of(75);
  public static final Angle STOW_POSITION = Rotations.of(10);
  public static final Angle STOW_SLOW_POSITION = Rotations.of(30);
  public static final Angle EXTEND_POSITION_LEFT = Rotations.of(106);
  public static final Angle EXTEND_POSITION_RIGHT = Rotations.of(105);

  // Positions for auto climb
  public static enum ClimbPosition {
    FRONT_LEFT(new Pose2d(1.54, 3.91, Rotation2d.kCW_90deg)),
    FRONT_RIGHT(new Pose2d(1.54, 3.50, Rotation2d.kCW_90deg)),
    BACK_LEFT(new Pose2d(0.67, 3.91, Rotation2d.kCCW_90deg)),
    BACK_RIGHT(new Pose2d(0.67, 3.50, Rotation2d.kCCW_90deg));

    private Pose2d pose;

    private ClimbPosition(Pose2d pose) {
      this.pose = pose;
    }

    public Pose2d getPose() {
      return pose;
    }
  }

  // Motor Config
  public static final MotorConfig MOTOR_CONFIG =
      switch (Constants.robot) {
        case REBUILT_PHOENIX, REBUILT_AKIT -> new MotorConfig(4, 5, true, Amps.of(80), 1.0 / 1.0);
        default -> new MotorConfig(4, 5, true, Amps.of(50), 1.0 / 1.0);
      };

  public record MotorConfig(
      int CANID_LEFT, int CANID_RIGHT, boolean INVERTED, Current MAX_CURRENT, double REDUCTION) {}
}
