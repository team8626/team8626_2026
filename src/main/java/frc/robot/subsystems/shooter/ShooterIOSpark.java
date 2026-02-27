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
import static frc.robot.util.SparkUtil.ifOk;
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
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.function.DoubleSupplier;

/** Shooter IO implementation for SPARK Flex motor controller. */
public class ShooterIOSpark implements ShooterIO {
  private final SparkFlex sparkLead;
  private final SparkFlex sparkFollow;
  private final SparkFlexConfig config;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;
  private final Debouncer connectedDebounce = new Debouncer(0.5);

  private double kV = ShooterConstants.velocityKv;
  private double kS = ShooterConstants.velocityKs;

  public ShooterIOSpark() {
    sparkLead = new SparkFlex(ShooterConstants.shooterLeadCanId, MotorType.kBrushless);
    sparkFollow = new SparkFlex(ShooterConstants.shooterFollowCanId, MotorType.kBrushless);
    encoder = sparkLead.getEncoder();
    controller = sparkLead.getClosedLoopController();

    // Configure motor
    config = new SparkFlexConfig();
    config
        .inverted(ShooterConstants.motorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ShooterConstants.motorCurrentLimit)
        .voltageCompensation(12.0);

    // Configure follower moter
    var followConfig = new SparkFlexConfig();
    followConfig.follow(
        sparkLead, true); // TODO: double check that right motor on shooter needs to be inverted

    // Configure encoder
    config
        .encoder
        .positionConversionFactor(ShooterConstants.encoderPositionFactor)
        .velocityConversionFactor(ShooterConstants.encoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    // Configure closed loop control
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ShooterConstants.velocityKp, 0.0, ShooterConstants.velocityKd, ClosedLoopSlot.kSlot0);

    // Configure signal update rates
    config
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    // Apply configuration
    tryUntilOk(
        sparkLead,
        5,
        () ->
            sparkLead.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(
        sparkFollow,
        5,
        () ->
            sparkFollow.configure(
                followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Reset encoder to zero position
    tryUntilOk(sparkLead, 5, () -> encoder.setPosition(0.0));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    sparkStickyFault = false;

    // Update velocities
    ifOk(
        sparkLead,
        encoder::getVelocity,
        (value) -> {
          inputs.velocityMotorLeft = RadiansPerSecond.of(value);
          inputs.velocityShooterWheel = RadiansPerSecond.of(value);
        });
    ifOk(
        sparkFollow,
        sparkFollow.getEncoder()::getVelocity,
        (value) -> inputs.velocityMotorRight = RadiansPerSecond.of(value));
    // Update appliedVoltage
    ifOk(
        sparkLead,
        new DoubleSupplier[] {sparkLead::getAppliedOutput, sparkLead::getBusVoltage},
        (values) -> inputs.appliedVoltageMotorLeft = Volts.of(values[0] * values[1]));
    ifOk(
        sparkFollow,
        new DoubleSupplier[] {sparkFollow::getAppliedOutput, sparkFollow::getBusVoltage},
        (values) -> inputs.appliedVoltageMotorRight = Volts.of(values[0] * values[1]));
    // Update current
    ifOk(
        sparkLead,
        sparkLead::getOutputCurrent,
        (value) -> inputs.currentMotorLeft = Amps.of(value));
    ifOk(
        sparkFollow,
        sparkFollow::getOutputCurrent,
        (value) -> inputs.currentMotorRight = Amps.of(value));

    inputs.connected = connectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setOpenLoop(double output) {
    sparkLead.setVoltage(output);
  }

  @Override
  public void setVelocity(AngularVelocity wheelVelocity) {
    // Use velocity PID slot (slot 0) with feedforward
    AngularVelocity motorVelocity = wheelVelocity.times(ShooterConstants.gearReduction);
    double ffVolts =
        kS * Math.signum(motorVelocity.in(RadiansPerSecond))
            + kV * motorVelocity.in(RadiansPerSecond);
    controller.setSetpoint(
        motorVelocity.in(RadiansPerSecond),
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  /*@Override //TODO: ask Mr. Dumet for removal
  public void setPosition(double positionRad) {
    // Use position PID slot (slot 1)
    controller.setReference(positionRad, ControlType.kPosition, ClosedLoopSlot.kSlot1);
  }*/

  @Override
  public void stop() {
    sparkLead.setVoltage(0.0);
  }

  @Override
  public void setPID(double new_kP, double new_kD, double new_kV, double new_kS) {
    config.closedLoop.pid(new_kP, 0.0, new_kD, ClosedLoopSlot.kSlot0);
    kV = new_kV;
    kS = new_kS;

    tryUntilOk(
        sparkLead,
        5,
        () ->
            sparkLead.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }
}
