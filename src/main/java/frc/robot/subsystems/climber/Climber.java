package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO io;

  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  // private final ClimberVisualizer visualizer = new ClimberVisualizer();

  private boolean disabled = false;

  private final Alert leftDisconnectedAlert =
      new Alert("Climber Left Motor Disconnected", AlertType.kError);
  private final Alert rightDisconnectedAlert =
      new Alert("Climber Right Motor Disconnected", AlertType.kError);

  public Climber(ClimberIO io) {
    this.io = io;

    // VirtualPD.registerMotor(() -> inputs.leftSupplyCurrent, "Climb");
    // VirtualPD.registerMotor(() -> inputs.rightSupplyCurrent, "Climb");

    SmartDashboard.putData("Climb/Climb", climb());
    SmartDashboard.putData("Climb/AutoClimb", autoClimb());
    SmartDashboard.putData("Climb/Stow", stow());
    SmartDashboard.putData("Climb/Extend", extend());
    SmartDashboard.putData("Climb/Zero", zero());

    SmartDashboard.putData("Overrides/Climber", disable());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    // visualizer.update(inputs.leftPosition, inputs.rightPosition);

    leftDisconnectedAlert.set(!inputs.leftConnected && Constants.currentMode != Constants.simMode);
    rightDisconnectedAlert.set(
        !inputs.rightConnected && Constants.currentMode != Constants.simMode);
  }

  /* Set motor voltage */
  private Command setVoltage(Voltage out) {
    return this.runOnce(() -> io.setVoltage(out));
  }

  /* Stop motors */
  private Command stop() {
    return this.runOnce(io::stop);
  }

  /* Checks if motors are in the extended position */
  public boolean isExtended() {
    return inputs.leftPosition.gte(ClimberConstants.EXTEND_POSITION_LEFT)
        && inputs.rightPosition.gte(ClimberConstants.EXTEND_POSITION_RIGHT);
  }

  /* * Commands executeable from SmartDashboard (may move later)*/

  /* Extends to a target position */
  public Command climb() {
    return Commands.sequence(
            setVoltage(ClimberConstants.CLIMB_VOLTAGE), // Set target voltage
            Commands.waitUntil(
                () ->
                    inputs.averagePosition.lte(
                        ClimberConstants
                            .CLIMB_POSITION)), // pauses until the average position is <= target //
            // higher position for slower decent (used at end of
            // teleop)
            // positon
            setVoltage(
                ClimberConstants
                    .CLIMB_LOCK_VOLTAGE), // Sets volts to a constant to "lock" the motors into
            // position
            Commands.idle()) // pauses command until interupted
        .onlyIf(() -> inputs.isZeroed)
        .finallyDo(io::stop) // stops motors
        .unless(() -> disabled); // negates last command if subsystem is disabled
  }

  /* Same command as climb(), but with a sepreate position */
  public Command autoClimb() {
    return Commands.sequence(
            setVoltage(ClimberConstants.CLIMB_VOLTAGE),
            Commands.waitUntil(
                () ->
                    inputs.averagePosition.lte(
                        ClimberConstants
                            .AUTO_CLIMB_POSITION)), // AUTO_CLIMB_POSITION provides a shorter climb
            // position for faster release from the tower
            // (used at end of auto)
            // positions
            setVoltage(ClimberConstants.CLIMB_LOCK_VOLTAGE),
            Commands.idle())
        .onlyIf(() -> inputs.isZeroed)
        .finallyDo(io::stop)
        .unless(() -> disabled);
  }

  /* stows the arms */
  public Command stow() {
    return Commands.sequence(
            setVoltage(ClimberConstants.STOW_VOLTAGE), // Set target voltage
            Commands.waitUntil(
                () ->
                    inputs.leftPosition.lte(ClimberConstants.STOW_SLOW_POSITION)
                        || inputs.rightPosition.lte(ClimberConstants.STOW_SLOW_POSITION)),
            // pauses until either motor has reached a
            // targeted position before completely
            // stowing
            setVoltage(ClimberConstants.STOW_SLOW_VOLTAGE), // Slows voltage on motors
            Commands
                .parallel( // Simaltaniously runs commands checking for each hook to return to the
                    // stow position, then stops once finished
                    Commands.waitUntil(
                            () -> inputs.leftPosition.lte(ClimberConstants.STOW_POSITION))
                        .finallyDo(io::stopLeft),
                    Commands.waitUntil(
                            () -> inputs.rightPosition.lte(ClimberConstants.STOW_POSITION))
                        .finallyDo(io::stopRight)),
            stop())
        .onlyIf(
            () ->
                inputs.isZeroed
                    && inputs.leftPosition.gte(ClimberConstants.STOW_POSITION)
                    && inputs.rightPosition.gte(ClimberConstants.STOW_POSITION))
        .finallyDo(io::stop) // finally stops the io (in case of interuption)
        .unless(() -> disabled || !inputs.isZeroed); // unless its disabled
  }

  /* extends the arms */
  public Command extend() {
    return Commands.sequence(
            setVoltage(ClimberConstants.EXTEND_VOLTAGE), // sets target voltage
            Commands.parallel( // Simaltaniously runs commands checking for each hook to reach the
                // extend position, then stops once finished
                Commands.waitUntil(
                        () -> inputs.leftPosition.gte(ClimberConstants.EXTEND_POSITION_LEFT))
                    .finallyDo(io::stopLeft),
                Commands.waitUntil(
                        () -> inputs.rightPosition.gte(ClimberConstants.EXTEND_POSITION_RIGHT))
                    .finallyDo(io::stopRight)),
            stop()) // finally stops the io (in case of interuption)
        .onlyIf(() -> inputs.isZeroed)
        .finallyDo(io::stop) // stops io
        .unless(() -> disabled); // unless disabled
  }

  /* zeros/resets the arms */
  public Command zero() {
    return Commands.sequence(
            this.runOnce(
                () -> {
                  inputs.isZeroed = false;
                  io.setLeftVoltage(ClimberConstants.ZERO_VOLTAGE);
                  io.setRightVoltage(ClimberConstants.ZERO_VOLTAGE);
                }), // sets each motor side to zero_voltage constant
            Commands.waitSeconds(0.1), // delay
            Commands
                .parallel( // Simaltaniously runs commands that wait until certain values are met
                    // and then stop the io
                    Commands.waitUntil(
                            () ->
                                inputs.leftCurrent.abs(Amps)
                                        > ClimberConstants.STALL_CURRENT.abs(
                                            Amps) // checks if current left current is greater than
                                    // the stall current (as absolute values)
                                    && inputs.leftVelocity.abs(RadiansPerSecond)
                                        < ClimberConstants.STALL_ANGULAR_VELOCITY.abs(
                                            RadiansPerSecond)) // checks if current left velocity is
                        // less than the stall velocity (as
                        // absolute values)
                        .finallyDo(() -> io.stopLeft()), // stops left motor
                    Commands.waitUntil( // same exact thing for right
                            () ->
                                inputs.rightCurrent.abs(Amps)
                                        > ClimberConstants.STALL_CURRENT.abs(Amps)
                                    && inputs.rightVelocity.abs(RadiansPerSecond)
                                        < ClimberConstants.STALL_ANGULAR_VELOCITY.abs(
                                            RadiansPerSecond))
                        .finallyDo(() -> io.stopRight())),
            Commands.waitSeconds(0.4), // delay
            this.runOnce(io::zeroPosition)) // zeros position variables (only in spark tho, not sim)
        .finallyDo(io::stop) // stops io
        .unless(() -> disabled); // unless disabled
  }

  /* manages the disabled boolean */
  public Command disable() {
    return this.runOnce(() -> disabled = true) // sets disabled to true
        .andThen(Commands.idle()) // idles command
        .finallyDo(() -> disabled = false) // sets disabled to false
        .withName("Disable Climber"); // names command
  }
}
