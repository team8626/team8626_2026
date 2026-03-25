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
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Voltage;

public class ClimberIOSpark implements ClimberIO {
  private final SparkFlex motorleft;
  private final RelativeEncoder encoderleft;
  private final SparkClosedLoopController controllerleft;

  private final SparkFlex motorright;
  private final RelativeEncoder encoderright;
  private final SparkClosedLoopController controllerright;

  private final Debouncer connectedDebounce = new Debouncer(0.5);
  // private Angle desiredAngle = IntakeLinkageConstants.STOW_ANGLE;
  private SparkFlexConfig configleft;
  private SparkFlexConfig configright;
  // private ArmFeedforward armFF = new ArmFeedforward(GAINS.kS(), GAINS.kG(), GAINS.kV());

  private double absoluteOffsetRotations = 0.0;

  private boolean leftIsEnabled = false;
  private boolean rightIsEnabled = false;

  public ClimberIOSpark() {

    motorleft = new SparkFlex(ClimberConstants.MOTOR_CONFIG.CANID_LEFT(), MotorType.kBrushless);
    encoderleft = motorleft.getEncoder();
    controllerleft = motorleft.getClosedLoopController();

    configleft = new SparkFlexConfig();

    // Motor: inversion, brake, current limit, voltage comp
    configleft
        .inverted(ClimberConstants.MOTOR_CONFIG.INVERTED())
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) ClimberConstants.MAX_CURRENT.in(Amps))
        .voltageCompensation(12.0);

    tryUntilOk(
        motorleft,
        5,
        () ->
            motorleft.configure(
                configleft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    motorright = new SparkFlex(ClimberConstants.MOTOR_CONFIG.CANID_RIGHT(), MotorType.kBrushless);
    encoderright = motorright.getEncoder();
    controllerright = motorright.getClosedLoopController();
    configright = new SparkFlexConfig();

    configright
        .inverted(!ClimberConstants.MOTOR_CONFIG.INVERTED()) // Motors are mirrored
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) ClimberConstants.MAX_CURRENT.in(Amps))
        .voltageCompensation(12.0);

    tryUntilOk(
        motorright,
        5,
        () ->
            motorright.configure(
                configright, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    sparkStickyFault = false;

    inputs.leftConnected = connectedDebounce.calculate(!sparkStickyFault);
    inputs.leftPosition = Rotations.of(encoderleft.getPosition());
    inputs.leftVelocity = RotationsPerSecond.of(encoderleft.getVelocity());
    inputs.leftCurrent = Amps.of(motorleft.getOutputCurrent());
    inputs.leftAppliedVoltage = Volts.of(motorleft.getAppliedOutput() * 12.0);

    inputs.rightConnected = connectedDebounce.calculate(!sparkStickyFault);
    inputs.rightPosition = Rotations.of(encoderright.getPosition());
    inputs.rightVelocity = RPM.of(encoderright.getVelocity());
    inputs.rightCurrent = Amps.of(motorright.getOutputCurrent());
    inputs.rightAppliedVoltage = Volts.of(motorright.getAppliedOutput() * 12.0);

    inputs.averagePosition = inputs.leftPosition.plus(inputs.rightPosition).div(2.0);
  }

  @Override
  public void setVoltage(Voltage out) {
    setLeftVoltage(out);
    setRightVoltage(out);
  }

  @Override
  public void setLeftVoltage(Voltage out) {
    controllerleft.setSetpoint(out.in(Volts), ControlType.kVoltage);

    leftIsEnabled = out.in(Volts) != 0;
  }

  @Override
  public void setRightVoltage(Voltage out) {
    controllerright.setSetpoint(out.in(Volts), ControlType.kVoltage);

    rightIsEnabled = out.in(Volts) != 0;
  }

  @Override
  public void stop() {
    stopLeft();
    stopRight();
  }

  @Override
  public void stopLeft() {
    controllerleft.setSetpoint(0, ControlType.kVoltage);
    leftIsEnabled = false;
  }

  @Override
  public void stopRight() {
    controllerright.setSetpoint(0, ControlType.kVoltage);
    rightIsEnabled = false;
  }

  @Override
  public void zeroPosition() {
    encoderleft.setPosition(0);
    encoderright.setPosition(0);
  }

  @Override
  public void setNeutralMode(NeutralModeValue neutralModeValue) {
    configleft.idleMode(IdleMode.kBrake);
    configright.idleMode(IdleMode.kBrake);

    motorleft.configure(
        configleft, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    motorright.configure(
        configright, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
