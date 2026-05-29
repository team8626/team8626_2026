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

package frc.robot.subsystems.intakeRoller;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intakeRoller.IntakeRollerConstants.GAINS;
import static frc.robot.subsystems.intakeRoller.IntakeRollerConstants.ROLLER_CONFIG;
import static frc.robot.util.PhoenixUtil.tryUntilOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.*;

/** Hardware IO for intakeRoller using a SPARK Flex. Velocity control only. */
public class IntakeRollerIOTalonFX implements IntakeRollerIO {
  private final TalonFX motor;
  private final TalonFXConfiguration config;
  private final Debouncer connectedDebounce = new Debouncer(0.5);

  private boolean isEnabled = false;

  /** Target WHEEL velocity for closed-loop control. */
  private AngularVelocity desiredWheelVelocity = RPM.of(0.0);

  /** Feedforward gains for velocity control. */
  SimpleMotorFeedforward intakeRollerFF =
      new SimpleMotorFeedforward(GAINS.kS(), GAINS.kV(), GAINS.kA());

  public IntakeRollerIOTalonFX() {
    motor = new TalonFX(ROLLER_CONFIG.CANID());
    config = new TalonFXConfiguration();

    // Motor: inversion, brake, current limit, voltage comp
    config
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(
                    ROLLER_CONFIG.INVERTED()
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast))
        .withCurrentLimits(
            new CurrentLimitsConfigs().withStatorCurrentLimit(ROLLER_CONFIG.MAX_CURRENT()));

    // Velocity PID + feedforward (only slot we use)
    config.withSlot0(new Slot0Configs().withKP(GAINS.kP()).withKI(GAINS.kI()).withKD(GAINS.kD()));

    tryUntilOk(5, () -> motor.getConfigurator().apply(config));
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    sparkStickyFault = false;

    inputs.velocityRPMMotor = motor.getVelocity().getValueAsDouble();
    inputs.velocityRPMRollers = inputs.velocityRPMMotor / (ROLLER_CONFIG.REDUCTION());
    inputs.velocityRPMDesired = desiredWheelVelocity.in(RPM);

    inputs.current = motor.getStatorCurrent().getValue();
    inputs.appliedVoltage = motor.getMotorVoltage().getValue();
    inputs.temperature = motor.getDeviceTemp().getValue();

    inputs.atGoal = motor.getClosedLoopError().getValueAsDouble() < 0.1;
    inputs.isEnabled = isEnabled;

    inputs.connected = connectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void start(AngularVelocity new_velocity) {
    desiredWheelVelocity = new_velocity;

    AngularVelocity motorAngularVelocity = desiredWheelVelocity.times(ROLLER_CONFIG.REDUCTION());

    motor.setControl(
        new VelocityVoltage(motorAngularVelocity)
            .withFeedForward(intakeRollerFF.calculate(motorAngularVelocity.in(RPM))));

    isEnabled = true;
  }

  public void stop() {
    desiredWheelVelocity = RPM.of(0.0);
    motor.setControl(new NeutralOut());
    isEnabled = false;
  }

  @Override
  public void setPID(double new_kP, double new_kI, double new_kD, double new_kV, double new_kS) {

    config.withSlot0(new Slot0Configs().withKP(new_kP).withKI(new_kI).withKD(new_kD));

    intakeRollerFF = new SimpleMotorFeedforward(new_kS, new_kV, GAINS.kA());

    tryUntilOk(5, () -> motor.getConfigurator().apply(config));
  }

  @Override
  public void setVoltage(double input) {
    motor.setVoltage(input);
  }
}
