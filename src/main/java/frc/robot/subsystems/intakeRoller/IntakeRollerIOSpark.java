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
import edu.wpi.first.units.measure.*;

/** Hardware IO for intakeRoller using a SPARK Flex. Velocity control only. */
public class IntakeRollerIOSpark implements IntakeRollerIO {
  private final SparkFlex spark;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;
  private final SparkFlexConfig config;
  private final Debouncer connectedDebounce = new Debouncer(0.5);

  private boolean isEnabled = false;

  /** Target WHEEL velocity for closed-loop control. */
  private AngularVelocity desiredWheelVelocity = RPM.of(0.0);

  /** Feedforward gains for velocity control. */
  SimpleMotorFeedforward intakeRollerFF =
      new SimpleMotorFeedforward(GAINS.kS(), GAINS.kV(), GAINS.kA());

  public IntakeRollerIOSpark() {
    spark = new SparkFlex(ROLLER_CONFIG.CANID(), MotorType.kBrushless);
    encoder = spark.getEncoder();
    controller = spark.getClosedLoopController();
    config = new SparkFlexConfig();

    // Motor: inversion, brake, current limit, voltage comp
    config
        .inverted(ROLLER_CONFIG.INVERTED())
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit((int) ROLLER_CONFIG.MAX_CURRENT().in(Amp))
        .voltageCompensation(12.0);

    // Encoder: mechanism radians and rad/s
    config
        .encoder
        // .positionConversionFactor(IntakeRollerConstants.encoderPositionFactor)
        // .velocityConversionFactor(IntakeRollerConstants.encoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2); // TODO: Why 2? Is this necessary?

    // Velocity PID + feedforward (only slot we use)
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(GAINS.kP(), GAINS.kI(), GAINS.kD(), ClosedLoopSlot.kSlot0)
        .outputRange(-1, 1);

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
  public void updateInputs(IntakeRollerIOInputs inputs) {
    sparkStickyFault = false;

    inputs.velocityRPMMotor = encoder.getVelocity();
    inputs.velocityRPMRollers = inputs.velocityRPMMotor / (ROLLER_CONFIG.REDUCTION());
    inputs.velocityRPMDesired = desiredWheelVelocity.in(RPM);

    inputs.current = Amps.of(spark.getOutputCurrent());
    inputs.appliedVoltage = Volts.of(spark.getAppliedOutput());
    inputs.temperature = Celsius.of(spark.getMotorTemperature());

    inputs.atGoal = controller.isAtSetpoint();
    inputs.isEnabled = isEnabled;

    inputs.connected = connectedDebounce.calculate(!sparkStickyFault);
    inputs.atGoal = controller.isAtSetpoint();
  }

  @Override
  public void setOpenLoop(Voltage output) {
    controller.setSetpoint(output.in(Volt), ControlType.kDutyCycle);

    if (output.in(Volt) != 0.0) {
      isEnabled = true;
    } else {
      isEnabled = false;
    }
  }

  @Override
  public void setVelocity(AngularVelocity new_velocity) {
    desiredWheelVelocity = new_velocity;

    AngularVelocity motorAngularVelocity = desiredWheelVelocity.times(ROLLER_CONFIG.REDUCTION());

    controller.setSetpoint(
        motorAngularVelocity.in(RPM),
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        intakeRollerFF.calculate(motorAngularVelocity.in(RPM)),
        ArbFFUnits.kVoltage);

    isEnabled = true;
  }

  public void stop() {
    desiredWheelVelocity = RPM.of(0.0);
    controller.setSetpoint(0.0, ControlType.kDutyCycle);
    isEnabled = false;
  }

  @Override
  public void setPID(double new_kP, double new_kI, double new_kD, double new_kV, double new_kS) {

    config.closedLoop.pid(new_kP, new_kI, new_kD, ClosedLoopSlot.kSlot0);

    intakeRollerFF = new SimpleMotorFeedforward(new_kS, new_kV, GAINS.kA());

    tryUntilOk(
        spark,
        5,
        () ->
            spark.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void setVoltage(double input) {
    spark.setVoltage(input);
  }
}
