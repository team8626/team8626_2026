package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class RumbleCommands {

  private static final double DEFAULT_INTENSITY = 0.75;
  private static final double PULSE_INTENSITY = 0.75;
  private static final double ALERT_INTENSITY = 1.0;
  private static final Time PULSE_ON_TIME = Seconds.of(0.2);
  private static final Time PULSE_OFF_TIME = Seconds.of(0.15);
  private static final Time PULSE_TIME = Seconds.of(2.0);
  private static final Time ALERT_TIME = Seconds.of(2.0);

  private RumbleCommands() {}

  private static void setBothRumble(XboxController controller, double intensity) {
    double clamped = Math.max(0.0, Math.min(1.0, intensity));
    controller.setRumble(XboxController.RumbleType.kLeftRumble, clamped);
    controller.setRumble(XboxController.RumbleType.kRightRumble, clamped);
  }

  private static void stopRumble(XboxController controller) {
    setBothRumble(controller, 0.0);
  }

  /** Continuous rumble. */
  public static Command Rumble(XboxController controller) {
    return Commands.startEnd(
            () -> setBothRumble(controller, DEFAULT_INTENSITY), () -> stopRumble(controller))
        .withName("RumbleContinuous");
  }

  /** Pulse rumble. */
  public static Command PulseRumble(XboxController controller, Time totalTime) {

    Command oneCycle =
        Commands.sequence(
            Commands.runOnce(() -> setBothRumble(controller, PULSE_INTENSITY)),
            Commands.waitTime(PULSE_ON_TIME),
            Commands.runOnce(() -> stopRumble(controller)),
            Commands.waitTime(PULSE_OFF_TIME));

    return oneCycle
        .repeatedly()
        .withTimeout(totalTime)
        .finallyDo(interrupted -> stopRumble(controller))
        .withName("RumblePulse");
  }

  public static Command PulseRumble(XboxController controller) {

    return PulseRumble(controller, PULSE_TIME);
  }

  /**
   * Alert Rumble.
   *
   * <p>Pattern: short-short-long-pause
   */
  public static Command AlertRumble(XboxController controller, Time totalTime) {
    Command pattern =
        Commands.sequence(
            Commands.runOnce(() -> setBothRumble(controller, ALERT_INTENSITY)),
            Commands.waitSeconds(0.10),
            Commands.runOnce(() -> stopRumble(controller)),
            Commands.waitSeconds(0.08),
            Commands.runOnce(() -> setBothRumble(controller, ALERT_INTENSITY)),
            Commands.waitSeconds(0.10),
            Commands.runOnce(() -> stopRumble(controller)),
            Commands.waitSeconds(0.08),
            Commands.runOnce(() -> setBothRumble(controller, ALERT_INTENSITY)),
            Commands.waitSeconds(0.35),
            Commands.runOnce(() -> stopRumble(controller)),
            Commands.waitSeconds(0.25));

    return pattern
        .repeatedly()
        .withTimeout(totalTime)
        .finallyDo(interrupted -> stopRumble(controller))
        .withName("RumbleAlert");
  }

  public static Command AlertRumble(XboxController controller) {
    return AlertRumble(controller, ALERT_TIME);
  }
}
