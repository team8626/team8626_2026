// Copyright 2026 FRC 8626
// http://github.com/team8626
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

package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

/** Index IO implementation for SPARK Flex motor controller. */
public class IndexerIOSpark implements IndexerIO {
  private final SparkFlex spark;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;
  private final Debouncer connectedDebounce = new Debouncer(0.5);

  public IndexerIOSpark() {
    spark = new SparkFlex(indexCanId, MotorType.kBrushless);
    encoder = spark.getEncoder();
    controller = spark.getClosedLoopController();

    // Configure motor
    var config = new SparkFlexConfig();
    config
        .inverted(motorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(motorCurrentLimit)
        .voltageCompensation(12.0);

    // Configure encoder
    config
        .encoder
        .positionConversionFactor(encoderPositionFactor)
        .velocityConversionFactor(encoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    // Configure closed loop control
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(velocityKp, 0.0, velocityKd, 0.0)
        .pidf(positionKp, 0.0, positionKd, 0.0, ClosedLoopSlot.kSlot1);

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
        spark,
        5,
        () ->
            spark.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Reset encoder to zero position
    tryUntilOk(spark, 5, () -> encoder.setPosition(0.0));
  }

  @Override
  public void updateInputs(IndexIOInputs inputs) {
    sparkStickyFault = false;

    ifOk(spark, encoder::getPosition, (value) -> inputs.positionRad = value);
    ifOk(spark, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        spark,
        new DoubleSupplier[] {spark::getAppliedOutput, spark::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(spark, spark::getOutputCurrent, (value) -> inputs.currentAmps = value);

    inputs.connected = connectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setOpenLoop(double output) {
    spark.setVoltage(output);
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    // Use velocity PID slot (slot 0) with feedforward
    double ffVolts = velocityKs * Math.signum(velocityRadPerSec) + velocityKv * velocityRadPerSec;
    controller.setReference(
        velocityRadPerSec,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setPosition(double positionRad) {
    // Use position PID slot (slot 1)
    controller.setReference(positionRad, ControlType.kPosition, ClosedLoopSlot.kSlot1);
  }

  @Override
  public void stop() {
    spark.setVoltage(0.0);
  }
}
