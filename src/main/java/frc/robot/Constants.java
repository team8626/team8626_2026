// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
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

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final RobotType robot = RobotType.SIMBOT;

  public static final Mode simMode = Mode.SIM;
  public static final Mode realMode = Mode.REAL;
  public static final Mode currentMode = RobotBase.isReal() ? realMode : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static enum RobotType {
    REBUILT_COMPBOT,
    TSUNAMI,
    SIMBOT
  }

  public static class RobotConstants {

    public static final CANBus CAN_FD_BUS = new CANBus("Bobby");
    public static final CANBus CAN_RIO_BUS = new CANBus("rio");
  }

  public static class Dimensions {
    public static final Distance BUMPER_THICKNESS; // frame to edge of bumper
    public static final Distance BUMPER_HEIGHT; // height from floor to top of bumper
    public static final Distance FRAME_SIZE_Y; // left to right (y-axis)
    public static final Distance FRAME_SIZE_X; // front to back (x-axis)

    static {
      switch (robot) {
        case REBUILT_COMPBOT, SIMBOT -> {
          BUMPER_THICKNESS = Inches.of(3.625);
          BUMPER_HEIGHT = Inches.of(7);
          FRAME_SIZE_Y = Inches.of(27.5);
          FRAME_SIZE_X = Inches.of(27);
        }
        case TSUNAMI -> {
          BUMPER_THICKNESS = Inches.of(4);
          BUMPER_HEIGHT = Inches.of(7);
          FRAME_SIZE_Y = Inches.of(27);
          FRAME_SIZE_X = Inches.of(27);
        }
        default -> throw new IllegalStateException("Unexpected robot: " + robot);
      }
    }

    public static final Distance FULL_WIDTH = FRAME_SIZE_Y.plus(BUMPER_THICKNESS.times(2));
    public static final Distance FULL_LENGTH = FRAME_SIZE_X.plus(BUMPER_THICKNESS.times(2));
  }

  public static class ControllerConstants {
    public static final int DRIVERPORT = 0;
    public static final int OPERATORPORT = 1;

    public static final double DEADBAND = 0.1;
  }

  public static class FieldConstants {
    public static final Distance FIELD_LENGTH = Inches.of(650.12);
    public static final Distance FIELD_WIDTH = Inches.of(316.64);

    public static final Distance ALLIANCE_ZONE = Inches.of(156.06);

    public static final Translation3d HUB_BLUE =
        new Translation3d(Inches.of(181.56), FIELD_WIDTH.div(2), Inches.of(56.4));
    public static final Translation3d HUB_RED =
        new Translation3d(
            FIELD_LENGTH.minus(Inches.of(181.56)), FIELD_WIDTH.div(2), Inches.of(56.4));
    public static final Distance FUNNEL_RADIUS = Inches.of(24);
    public static final Distance FUNNEL_HEIGHT = Inches.of(72 - 56.4);

    private static final Distance TRENCH_BUMP_X = Inches.of(181.56);
    private static final Distance TRENCH_WIDTH = Inches.of(49.86);
    private static final Distance BUMP_INSET = TRENCH_WIDTH.plus(Inches.of(12));
    private static final Distance BUMP_LENGTH = Inches.of(73);

    private static final Distance TRENCH_ZONE_EXTENSION = Inches.of(70);
    private static final Distance BUMP_ZONE_EXTENSION = Inches.of(60);
    private static final Distance TRENCH_BUMP_ZONE_TRANSITION =
        TRENCH_WIDTH.plus(BUMP_INSET).div(2);

    public static final Translation2d[][] TRENCH_ZONES = {
      new Translation2d[] {
        new Translation2d(TRENCH_BUMP_X.minus(TRENCH_ZONE_EXTENSION), Inches.zero()),
        new Translation2d(TRENCH_BUMP_X.plus(TRENCH_ZONE_EXTENSION), TRENCH_BUMP_ZONE_TRANSITION)
      },
      new Translation2d[] {
        new Translation2d(
            TRENCH_BUMP_X.minus(TRENCH_ZONE_EXTENSION),
            FIELD_WIDTH.minus(TRENCH_BUMP_ZONE_TRANSITION)),
        new Translation2d(TRENCH_BUMP_X.plus(TRENCH_ZONE_EXTENSION), FIELD_WIDTH)
      },
      new Translation2d[] {
        new Translation2d(
            FIELD_LENGTH.minus(TRENCH_BUMP_X.plus(TRENCH_ZONE_EXTENSION)), Inches.zero()),
        new Translation2d(
            FIELD_LENGTH.minus(TRENCH_BUMP_X.minus(TRENCH_ZONE_EXTENSION)),
            TRENCH_BUMP_ZONE_TRANSITION)
      },
      new Translation2d[] {
        new Translation2d(
            FIELD_LENGTH.minus(TRENCH_BUMP_X.plus(TRENCH_ZONE_EXTENSION)),
            FIELD_WIDTH.minus(TRENCH_BUMP_ZONE_TRANSITION)),
        new Translation2d(
            FIELD_LENGTH.minus(TRENCH_BUMP_X.minus(TRENCH_ZONE_EXTENSION)), FIELD_WIDTH)
      }
    };

    public static final Translation2d[][] BUMP_ZONES = {
      new Translation2d[] {
        new Translation2d(TRENCH_BUMP_X.minus(BUMP_ZONE_EXTENSION), TRENCH_BUMP_ZONE_TRANSITION),
        new Translation2d(TRENCH_BUMP_X.plus(BUMP_ZONE_EXTENSION), BUMP_INSET.plus(BUMP_LENGTH))
      },
      new Translation2d[] {
        new Translation2d(
            TRENCH_BUMP_X.minus(BUMP_ZONE_EXTENSION),
            FIELD_WIDTH.minus(BUMP_INSET.plus(BUMP_LENGTH))),
        new Translation2d(
            TRENCH_BUMP_X.plus(BUMP_ZONE_EXTENSION), FIELD_WIDTH.minus(TRENCH_BUMP_ZONE_TRANSITION))
      },
      new Translation2d[] {
        new Translation2d(
            FIELD_LENGTH.minus(TRENCH_BUMP_X.plus(BUMP_ZONE_EXTENSION)),
            FIELD_WIDTH.minus(BUMP_INSET.plus(BUMP_LENGTH))),
        new Translation2d(
            FIELD_LENGTH.minus(TRENCH_BUMP_X.minus(BUMP_ZONE_EXTENSION)),
            FIELD_WIDTH.minus(TRENCH_BUMP_ZONE_TRANSITION))
      },
      new Translation2d[] {
        new Translation2d(
            FIELD_LENGTH.minus(TRENCH_BUMP_X.plus(BUMP_ZONE_EXTENSION)),
            TRENCH_BUMP_ZONE_TRANSITION),
        new Translation2d(
            FIELD_LENGTH.minus(TRENCH_BUMP_X.minus(BUMP_ZONE_EXTENSION)),
            BUMP_INSET.plus(BUMP_LENGTH))
      }
    };

    public static final Distance TRENCH_CENTER = TRENCH_WIDTH.div(2);
  }
}
