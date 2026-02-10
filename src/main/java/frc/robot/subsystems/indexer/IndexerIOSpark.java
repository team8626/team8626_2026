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

package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.encoderPositionFactor;
import static frc.robot.subsystems.indexer.IndexerConstants.encoderVelocityFactor;
import static frc.robot.subsystems.indexer.IndexerConstants.indexCanId;
import static frc.robot.subsystems.indexer.IndexerConstants.motorCurrentLimit;
import static frc.robot.subsystems.indexer.IndexerConstants.motorInverted;
import static frc.robot.subsystems.indexer.IndexerConstants.velocityKd;
import static frc.robot.subsystems.indexer.IndexerConstants.velocityKp;
import static frc.robot.subsystems.indexer.IndexerConstants.velocityKs;
import static frc.robot.subsystems.indexer.IndexerConstants.velocityKv;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

/** Hardware IO for indexer using a SPARK Flex. Velocity control only. */
public class IndexerIOSpark implements IndexerIO {
  private final SparkFlex spark;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;
  private final Debouncer connectedDebounce = new Debouncer(0.5);

  public IndexerIOSpark() {
    spark = new SparkFlex(indexCanId, MotorType.kBrushless);
    encoder = spark.getEncoder();
    controller = spark.getClosedLoopController();

    SparkFlexConfig config = new SparkFlexConfig();

    // Motor: inversion, brake, current limit, voltage comp
    config
        .inverted(motorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(motorCurrentLimit)
        .voltageCompensation(12.0);

    // Encoder: mechanism radians and rad/s
    config
        .encoder
        .positionConversionFactor(encoderPositionFactor)
        .velocityConversionFactor(encoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    // Velocity PID + feedforward (only slot we use)
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(velocityKp, 0.0, velocityKd, 0.0);

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
        spark,
        5,
        () ->
            spark.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

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
    double ffVolts = velocityKs * Math.signum(velocityRadPerSec) + velocityKv * velocityRadPerSec;
    controller.setReference(
        velocityRadPerSec,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    spark.setVoltage(0.0);
  }
}
