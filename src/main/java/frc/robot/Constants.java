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

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final RobotType robot = RobotType.SIMBOT;

  public static final boolean tuningMode = false;

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

  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  /** Checks whether the correct robot is selected when deploying. */
  public static class CheckDeploy {
    public static void main(String... args) {
      if (robot == RobotType.SIMBOT) {
        System.err.println("Cannot deploy, invalid robot selected: " + robot);
        System.exit(1);
      }
    }
  }

  /** Checks that the default robot is selected and tuning mode is disabled. */
  public static class CheckPullRequest {
    public static void main(String... args) {
      if (robot != RobotType.REBUILT_COMPBOT || tuningMode) {
        System.err.println("Do not merge, non-default constants are configured.");
        System.exit(1);
      }
    }
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
}
