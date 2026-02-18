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
import edu.wpi.first.units.measure.*;
import java.util.function.DoubleSupplier;

/** Hardware IO for indexer using a SPARK Flex. Velocity control only. */
public class IndexerIOSpark implements IndexerIO {
  private final SparkFlex spark;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;
  private final SparkFlexConfig config;
  private final Debouncer connectedDebounce = new Debouncer(0.5);

  /** Target WHEEL velocity for closed-loop control. */
  private AngularVelocity desiredWheelVelocity = RPM.of(0.0);

  /** Feedforward gains for velocity control. */
  private double kV = IndexerConstants.velocityKv;

  private double kS = IndexerConstants.velocityKs;

  public IndexerIOSpark() {
    spark = new SparkFlex(IndexerConstants.indexCanId, MotorType.kBrushless);
    encoder = spark.getEncoder();
    controller = spark.getClosedLoopController();
    config = new SparkFlexConfig();

    // Motor: inversion, brake, current limit, voltage comp
    config
        .inverted(IndexerConstants.motorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IndexerConstants.motorCurrentLimit)
        .voltageCompensation(12.0);

    // Encoder: mechanism radians and rad/s
    config
        .encoder
        .positionConversionFactor(IndexerConstants.encoderPositionFactor)
        .velocityConversionFactor(IndexerConstants.encoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2); // TODO: Why 2? Is this necessary?

    // Velocity PID + feedforward (only slot we use)
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(IndexerConstants.velocityKp, 0.0, IndexerConstants.velocityKd, ClosedLoopSlot.kSlot0);

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

    ifOk(
        spark,
        encoder::getVelocity,
        (value) -> inputs.actualWheelVelocity = RadiansPerSecond.of(value));
    ifOk(
        spark,
        new DoubleSupplier[] {spark::getAppliedOutput, spark::getBusVoltage},
        (values) -> inputs.appliedVoltage = Volts.of(values[0] * values[1]));
    ifOk(spark, spark::getOutputCurrent, (value) -> inputs.current = Amps.of(value));

    inputs.connected = connectedDebounce.calculate(!sparkStickyFault);
    inputs.desiredWheelVelocity = desiredWheelVelocity;
    inputs.atGoal =
        Math.abs(inputs.actualWheelVelocity.in(RPM) - inputs.desiredWheelVelocity.in(RPM))
            < IndexerConstants.VELOCITY_TOLERANCE.in(RPM);
  }

  @Override
  public void setOpenLoop(Voltage output) {
    spark.setVoltage(output);
  }

  @Override
  public void setVelocity(AngularVelocity new_velocity) {
    desiredWheelVelocity = new_velocity;

    AngularVelocity motorAngularVelocity =
        desiredWheelVelocity.times(IndexerConstants.GEAR_REDUCTION);

    double ffVolts =
        kS * Math.signum(motorAngularVelocity.in(RadiansPerSecond))
            + kV * motorAngularVelocity.in(RadiansPerSecond);

    controller.setSetpoint(
        motorAngularVelocity.in(RadiansPerSecond),
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  public void stop() {
    desiredWheelVelocity = RPM.of(0.0);
    controller.setSetpoint(0.0, ControlType.kDutyCycle);
  }

  @Override
  public void setPID(double new_kP, double new_kD, double new_kV, double new_kS) {

    config.closedLoop.pid(new_kP, 0.0, new_kD, ClosedLoopSlot.kSlot0);
    kV = new_kV;
    kS = new_kS;

    tryUntilOk(
        spark,
        5,
        () ->
            spark.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }
}
