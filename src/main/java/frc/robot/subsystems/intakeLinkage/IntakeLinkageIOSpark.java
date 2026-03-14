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

package frc.robot.subsystems.intakeLinkage;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intakeLinkage.IntakeLinkageConstants.GAINS;
import static frc.robot.subsystems.intakeLinkage.IntakeLinkageConstants.MOTOR_CONFIG;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;

/** Hardware IO for IntakeLinkage using a SPARK Flex. Position control (closed-loop). */
public class IntakeLinkageIOSpark implements IntakeLinkageIO {
  private final SparkFlex motor;
  private final AbsoluteEncoder encoder;
  private final SparkClosedLoopController controller;
  private final Debouncer connectedDebounce = new Debouncer(0.5);
  private Angle desiredAngle = IntakeLinkageConstants.STOW_ANGLE;
  private SparkFlexConfig config;
  private ArmFeedforward armFF = new ArmFeedforward(GAINS.kS(), GAINS.kG(), GAINS.kV());

  // private boolean isEnabled = false;
  // For AbsoluteEncoder: store a software offset (in rotations) captured at startup
  // so readings can be reported relative to that startup 'zero'. Absolute encoders
  // are read-only so we cannot call setPosition on them.
  private double absoluteOffsetRotations = 0.0;
  private boolean isEnabled = false;

  public IntakeLinkageIOSpark() {

    motor = new SparkFlex(IntakeLinkageConstants.INTAKE_LINKAGE_CAN_ID, MotorType.kBrushless);
    encoder = motor.getAbsoluteEncoder();
    controller = motor.getClosedLoopController();

    config = new SparkFlexConfig();

    // Motor: inversion, brake, current limit, voltage comp
    config
        .inverted(MOTOR_CONFIG.INVERTED())
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) MOTOR_CONFIG.MAX_CURRENT().in(Amps))
        .voltageCompensation(12.0); // TODO:add 12V voltage compensation to constants and config

    // Encoder: mechanism  deg and deg/s (via conversion factors)
    config
        .absoluteEncoder
        .positionConversionFactor(IntakeLinkageConstants.ENCODER_POSITION_FACTOR)
        .velocityConversionFactor(IntakeLinkageConstants.ENCODER_VELOCITY_FACTOR)
        .inverted(true);

    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(GAINS.kP(), GAINS.kI(), GAINS.kD(), ClosedLoopSlot.kSlot0)
        .feedForward
        .kCosRatio(0.0); // TODO: Tune feedforward (gravity compensation) if needed

    // Logging: encoder and output at 20 ms
    config
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Absolute encoders are read-only; capture the current absolute reading once
    // and treat it as the software zero for subsequent operations.
    ifOk(motor, encoder::getPosition, (value) -> absoluteOffsetRotations = value);
  }

  @Override
  public void updateInputs(IntakeLinkageIOInputs inputs) {
    sparkStickyFault = false;

    inputs.position = Degrees.of(encoder.getPosition());
    inputs.appliedVoltage = Volts.of(motor.getBusVoltage());
    inputs.current = Amps.of(motor.getOutputCurrent());
    inputs.desiredAngle = desiredAngle;
    inputs.atGoal = controller.isAtSetpoint();
    inputs.isEnabled = !inputs.atGoal && controller.getControlType() == ControlType.kPosition;

    inputs.positionRad = inputs.position.in(Radians);
    inputs.velocityRadPerSec = inputs.velocity.in(RadiansPerSecond);

    inputs.connected = connectedDebounce.calculate(!sparkStickyFault);
    controller.setSetpoint(
        desiredAngle.in(Degrees),
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        armFF.calculate(desiredAngle.in(Degrees), inputs.velocity.in(DegreesPerSecond)),
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setPosition(Angle position) {
    desiredAngle = position;
  }

  public void setPID(double new_kP, double new_kI, double new_kD) {
    config.closedLoop.pid(new_kP, new_kI, new_kD, ClosedLoopSlot.kSlot0);
    // set offset in motor controller so that the current position becomes the new zero
    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public void setPID(
      double new_kP, double new_kI, double new_kD, double new_kV, double new_kG, double new_kS) {
    setPID(new_kP, new_kI, new_kD);
    armFF.setKv(new_kV);
    armFF.setKg(new_kG);
    armFF.setKs(new_kS);
  }

  @Override
  public void runCharacterization(double input) {
    motor.setVoltage(input);
  }
}
