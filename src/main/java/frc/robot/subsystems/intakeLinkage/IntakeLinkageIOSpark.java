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

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
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
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import java.util.function.DoubleSupplier;

/** Hardware IO for IntakeLinkage using a SPARK Flex. Position control (closed-loop). */
public class IntakeLinkageIOSpark implements IntakeLinkageIO {
  private final SparkFlex spark;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;
  private final Debouncer connectedDebounce = new Debouncer(0.5);
  private Angle desiredAngle = Radians.of(0.0);

  public IntakeLinkageIOSpark() {
    spark = new SparkFlex(IntakeLinkageConstants.intakeLinkageCanId, MotorType.kBrushless);
    encoder = spark.getEncoder();
    controller = spark.getClosedLoopController();

    SparkFlexConfig config = new SparkFlexConfig();

    // Motor: inversion, brake, current limit, voltage comp
    config
        .inverted(IntakeLinkageConstants.motorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeLinkageConstants.motorCurrentLimit)
        .voltageCompensation(12.0);

    // Encoder: mechanism radians and rad/s (via conversion factors)
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
            0.0,
            IntakeLinkageConstants.positionKd,
            ClosedLoopSlot.kSlot0);

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
  public void updateInputs(IntakeLinkageIOInputs inputs) {
    sparkStickyFault = false;

    ifOk(spark, encoder::getPosition, (value) -> inputs.position = Radians.of(value));
    ifOk(spark, encoder::getVelocity, (value) -> inputs.velocity = RadiansPerSecond.of(value));

    ifOk(
        spark,
        new DoubleSupplier[] {spark::getAppliedOutput, spark::getBusVoltage},
        (values) -> inputs.appliedVoltage = Volts.of(values[0] * values[1]));

    ifOk(spark, spark::getOutputCurrent, (value) -> inputs.current = Amps.of(value));
    inputs.desiredAngle = desiredAngle;

    // ifOk(spark, spark::getMotorTemperature, (value) -> inputs.temperature = Celsius.of(value));

    inputs.connected = connectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setPosition(Angle position) {
    desiredAngle = position;

    // Slot 0 = position PID
    controller.setSetpoint(position.in(Radians), ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void setVoltage(Voltage volts) {
    spark.setVoltage(volts);
  }

  @Override
  public void stop() {
    setVoltage(Volts.of(0.0));
  }
}
