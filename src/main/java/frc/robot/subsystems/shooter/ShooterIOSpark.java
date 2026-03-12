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

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;

/** Shooter IO implementation for SPARK Flex motor controller. */
public class ShooterIOSpark implements ShooterIO {
  private final SparkFlex motorLeft;
  private final SparkFlex motorRight;

  private final SparkFlexConfig configLeft;
  private final RelativeEncoder encoderLeft;
  private final SparkClosedLoopController controllerLeft;

  private final Debouncer connectedDebounce = new Debouncer(0.5);

  private AngularVelocity desiredVelocity = RPM.of(0.0);

  private double kV = ShooterConstants.SHOOTER_KV;
  private double kS = ShooterConstants.SHOOTER_KS;

  private SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(kS, kV, 0.0);

  public ShooterIOSpark() {
    // Initialize motors and related objects
    motorLeft = new SparkFlex(ShooterConstants.LEFT_CANID, MotorType.kBrushless);
    motorRight = new SparkFlex(ShooterConstants.RIGHT_CANID, MotorType.kBrushless);

    encoderLeft = motorLeft.getEncoder();
    controllerLeft = motorLeft.getClosedLoopController();

    // Configure motor
    configLeft = new SparkFlexConfig();
    configLeft
        .inverted(ShooterConstants.SHOOTER_LEFT_INVERTED)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit((int) ShooterConstants.CURRENT_LIMIT.in(Amps))
        .voltageCompensation(12.0);

    // Configure encoder
    configLeft
        .encoder
        .positionConversionFactor(1.0 / ShooterConstants.GEAR_REDUCTION)
        .velocityConversionFactor(1.0 / ShooterConstants.GEAR_REDUCTION)
        .uvwMeasurementPeriod(10);

    // Configure closed loop control
    configLeft
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(
            ShooterConstants.SHOOTER_KP,
            ShooterConstants.SHOOTER_KI,
            ShooterConstants.SHOOTER_KD,
            ClosedLoopSlot.kSlot0)
        .outputRange(-1.0, 1.0);

    // Configure signal update rates
    // config
    //     .signals
    //     .primaryEncoderPositionAlwaysOn(true)
    //     .primaryEncoderPositionPeriodMs(20)
    //     .primaryEncoderVelocityAlwaysOn(true)
    //     .primaryEncoderVelocityPeriodMs(20)
    //     .appliedOutputPeriodMs(20)
    //     .busVoltagePeriodMs(20)
    //     .outputCurrentPeriodMs(20);

    // Configure right motor as a follower (inverted)
    var configRight = new SparkFlexConfig();
    configRight.follow(motorLeft, true);

    // Apply configuration
    tryUntilOk(
        motorLeft,
        5,
        () ->
            motorLeft.configure(
                configLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(
        motorRight,
        5,
        () ->
            motorRight.configure(
                configRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Reset encoder to zero position
    tryUntilOk(motorLeft, 5, () -> encoderLeft.setPosition(0.0));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    sparkStickyFault = false;

    // Update Currents
    inputs.currentMotorLeft = Amps.of(motorLeft.getOutputCurrent());
    inputs.currentMotorRight = Amps.of(motorRight.getOutputCurrent());

    // Update Voltages
    inputs.appliedVoltageMotorLeft = Volts.of(motorLeft.getAppliedOutput());
    inputs.appliedVoltageMotorRight = Volts.of(motorRight.getAppliedOutput());

    // Update velocities
    inputs.velocityMotorLeft = RPM.of(encoderLeft.getVelocity());
    inputs.velocityMotorRight = RPM.of(motorRight.getEncoder().getVelocity());
    inputs.velocityShooterWheel = RPM.of(encoderLeft.getVelocity());
    inputs.desiredWheelVelocity = desiredVelocity;

    // Update position
    inputs.positionMotorLeft = Radians.of(encoderLeft.getPosition());

    // Update velocities
    // ifOk(
    //     motorLeft,
    //     encoderLeft::getVelocity,
    //     (value) -> {
    //       inputs.velocityMotorLeft = RadiansPerSecond.of(value);
    //       inputs.velocityShooterWheel = RadiansPerSecond.of(value);
    //     });
    // ifOk(
    //     motorRight,
    //     motorRight.getEncoder()::getVelocity,
    //     (value) -> inputs.velocityMotorRight = RadiansPerSecond.of(value));
    // // Update appliedVoltage
    // ifOk(
    //     motorLeft,
    //     new DoubleSupplier[] {motorLeft::getAppliedOutput, motorLeft::getBusVoltage},
    //     (values) -> inputs.appliedVoltageMotorLeft = Volts.of(values[0] * values[1]));
    // ifOk(
    //     motorRight,
    //     new DoubleSupplier[] {motorRight::getAppliedOutput, motorRight::getBusVoltage},
    //     (values) -> inputs.appliedVoltageMotorRight = Volts.of(values[0] * values[1]));
    // // Update current
    // ifOk(
    //     motorLeft,
    //     motorLeft::getOutputCurrent,
    //     (value) -> inputs.currentMotorLeft = Amps.of(value));
    // ifOk(
    //     motorRight,
    //     motorRight::getOutputCurrent,
    //     (value) -> inputs.currentMotorRight = Amps.of(value));

    inputs.connected = connectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setOpenLoop(double output) {
    controllerLeft.setSetpoint(output, ControlType.kDutyCycle);
    desiredVelocity = RPM.of(0); // Setpoint is zero since we're controlling voltage directly
  }

  @Override
  public void setVelocity(AngularVelocity new_wheelVelocity) {
    desiredVelocity = new_wheelVelocity;

    // Apply gear reduction to get motor setpoint
    // AngularVelocity motorVelocity = new_wheelVelocity.times(ShooterConstants.GEAR_REDUCTION);

    controllerLeft.setSetpoint(
        desiredVelocity.in(RPM),
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        shooterFF.calculate(desiredVelocity.in(RPM)),
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    controllerLeft.setSetpoint(0.0, ControlType.kDutyCycle);
    desiredVelocity = RPM.of(0);
  }

  @Override
  public void setPID(double new_kP, double new_kI, double new_kD, double new_kV, double new_kS) {
    configLeft.closedLoop.pid(new_kP, new_kI, new_kD, ClosedLoopSlot.kSlot0);
    kV = new_kV;
    kS = new_kS;

    tryUntilOk(
        motorLeft,
        5,
        () ->
            motorLeft.configure(
                configLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }
}
