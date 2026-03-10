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
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import java.util.function.DoubleSupplier;

/** Hardware IO for IntakeLinkage using a SPARK Flex. Position control (closed-loop). */
public class IntakeLinkageIOSpark implements IntakeLinkageIO {
  private final SparkFlex motor;
  private final AbsoluteEncoder encoder;
  private final SparkClosedLoopController controller;
  private final Debouncer connectedDebounce = new Debouncer(0.5);
  private Angle desiredAngle;
  private SparkFlexConfig config;

  // private boolean isEnabled = false;
  // For AbsoluteEncoder: store a software offset (in rotations) captured at startup
  // so readings can be reported relative to that startup 'zero'. Absolute encoders
  // are read-only so we cannot call setPosition on them.
  private double absoluteOffsetRotations = 0.0;

  public IntakeLinkageIOSpark() {

    motor = new SparkFlex(IntakeLinkageConstants.intakeLinkageCanId, MotorType.kBrushless);
    encoder = motor.getAbsoluteEncoder();
    controller = motor.getClosedLoopController();

    config = new SparkFlexConfig();

    // Motor: inversion, brake, current limit, voltage comp
    config
        .inverted(IntakeLinkageConstants.motorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeLinkageConstants.motorCurrentLimit)
        .voltageCompensation(12.0); // TODO:add 12V voltage compensation to constants and config

    // Encoder: mechanism  deg and deg/s (via conversion factors)
    config
        .encoder
        .positionConversionFactor(IntakeLinkageConstants.encoderPositionFactor)
        .velocityConversionFactor(IntakeLinkageConstants.encoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    // Closed-loop: Position PID in Slot 0
    // NOTE: Add these constants in IntakeLinkageConstants:
    //   public static final double positionKp = ...;
    //   public static final double positionKd = ...;
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(
            IntakeLinkageConstants.positionKp,
            IntakeLinkageConstants.positionKi,
            IntakeLinkageConstants.positionKd,
            ClosedLoopSlot.kSlot0)
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

    // Report position relative to the startup zero offset we captured earlier.
    ifOk(
        motor,
        encoder::getPosition,
        (value) -> inputs.position = Rotations.of(value - absoluteOffsetRotations));
    ifOk(motor, encoder::getVelocity, (value) -> inputs.velocity = RotationsPerSecond.of(value));

    ifOk(
        motor,
        new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
        (values) -> inputs.appliedVoltage = Volts.of(values[0] * values[1]));

    ifOk(motor, motor::getOutputCurrent, (value) -> inputs.current = Amps.of(value));
    inputs.desiredAngle = desiredAngle;

    // ifOk(motor, motor::getMotorTemperature, (value) -> inputs.temperature = Celsius.of(value));

    inputs.connected = connectedDebounce.calculate(!sparkStickyFault);
    controller.setSetpoint(desiredAngle.in(Degrees), ControlType.kPosition);
  }

  @Override
  public void setPosition(Angle position) {
    desiredAngle = position;

    // Slot 0 = position PID. Add the startup offset (converted to degrees) so the
    // controller sees values in the same absolute-sensor frame we report.
    // double offsetDegrees = Rotations.of(absoluteOffsetRotations).in(Degrees);
    // controller.setSetpoint(
    //    position.in(Degrees) + offsetDegrees, ControlType.kPosition, ClosedLoopSlot.kSlot0);
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

  @Override
  public void stow() {
    desiredAngle = IntakeLinkageConstants.STOW_ANGLE;
  }

  @Override
  public void deploy() {
    desiredAngle = IntakeLinkageConstants.DEPLOY_ANGLE;
  }

  @Override
  public void hopperOpen() {
    desiredAngle = IntakeLinkageConstants.HOPPER_OPEN_ANGLE;
  }
}
