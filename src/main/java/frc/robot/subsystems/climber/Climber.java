package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

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

  private Command setVoltage(Voltage out) {
    return this.runOnce(() -> io.setVoltage(out));
  }

  private Command stop() {
    return this.runOnce(io::stop);
  }

  public boolean isExtended() {
    return inputs.leftPosition.gte(ClimberConstants.EXTEND_POSITION_LEFT)
        && inputs.rightPosition.gte(ClimberConstants.EXTEND_POSITION_RIGHT);
  }

  public Command climb() {
    return Commands.sequence(
            setVoltage(ClimberConstants.CLIMB_VOLTAGE),
            Commands.waitUntil(() -> inputs.averagePosition.lte(ClimberConstants.CLIMB_POSITION)),
            setVoltage(Volts.of(-0.5)),
            Commands.idle())
        .finallyDo(io::stop)
        .unless(() -> disabled);
  }

  public Command autoClimb() {
    return Commands.sequence(
            setVoltage(ClimberConstants.CLIMB_VOLTAGE),
            Commands.waitUntil(
                () -> inputs.averagePosition.lte(ClimberConstants.AUTO_CLIMB_POSITION)),
            setVoltage(Volts.of(-0.5)),
            Commands.idle())
        .finallyDo(io::stop)
        .unless(() -> disabled);
  }

  public Command stow() {
    return Commands.sequence(
            setVoltage(ClimberConstants.STOW_VOLTAGE),
            Commands.waitUntil(
                () ->
                    inputs.leftPosition.lte(ClimberConstants.STOW_SLOW_POSITION)
                        || inputs.rightPosition.lte(ClimberConstants.STOW_SLOW_POSITION)),
            setVoltage(ClimberConstants.STOW_SLOW_VOLTAGE),
            Commands.parallel(
                Commands.waitUntil(() -> inputs.leftPosition.lte(ClimberConstants.STOW_POSITION))
                    .finallyDo(io::stopLeft),
                Commands.waitUntil(() -> inputs.rightPosition.lte(ClimberConstants.STOW_POSITION))
                    .finallyDo(io::stopRight)),
            stop())
        .finallyDo(io::stop)
        .unless(() -> disabled);
  }

  public Command extend() {
    return Commands.sequence(
            setVoltage(ClimberConstants.EXTEND_VOLTAGE),
            Commands.parallel(
                Commands.waitUntil(
                        () -> inputs.leftPosition.gte(ClimberConstants.EXTEND_POSITION_LEFT))
                    .finallyDo(io::stopLeft),
                Commands.waitUntil(
                        () -> inputs.rightPosition.gte(ClimberConstants.EXTEND_POSITION_RIGHT))
                    .finallyDo(io::stopRight)),
            stop())
        .finallyDo(io::stop)
        .unless(() -> disabled);
  }

  public Command zero() {
    return Commands.sequence(
            this.runOnce(
                () -> {
                  io.setLeftVoltage(ClimberConstants.ZERO_VOLTAGE);
                  io.setRightVoltage(ClimberConstants.ZERO_VOLTAGE);
                }),
            Commands.waitSeconds(0.1),
            Commands.parallel(
                Commands.waitUntil(
                        () ->
                            inputs.leftCurrent.abs(Amps) > ClimberConstants.STALL_CURRENT.abs(Amps)
                                && inputs.leftVelocity.abs(RadiansPerSecond)
                                    < ClimberConstants.STALL_ANGULAR_VELOCITY.abs(RadiansPerSecond))
                    .finallyDo(() -> io.stopLeft()),
                Commands.waitUntil(
                        () ->
                            inputs.rightCurrent.abs(Amps) > ClimberConstants.STALL_CURRENT.abs(Amps)
                                && inputs.rightVelocity.abs(RadiansPerSecond)
                                    < ClimberConstants.STALL_ANGULAR_VELOCITY.abs(RadiansPerSecond))
                    .finallyDo(() -> io.stopRight())),
            Commands.waitSeconds(0.4),
            this.runOnce(io::zeroPosition))
        .finallyDo(io::stop)
        .unless(() -> disabled);
  }

  public Command disable() {
    return this.runOnce(() -> disabled = true)
        .andThen(Commands.idle())
        .finallyDo(() -> disabled = false)
        .withName("Disable Climber");
  }
}
