package frc.robot.subsystems.anotherShooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.subsystems.anotherShooter.AnotherShooterConstants.FLYWHEEL_CONFIG;
import static frc.robot.subsystems.anotherShooter.AnotherShooterConstants.GAINS;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.anotherShooter.AnotherShooterIO.AnotherShooterIOInputs;

public class AnotherShooterIOSparkFlex implements AnotherShooterIO {

  private final SparkFlex leftMotor;
  private final SparkFlexConfig leftConfig;
  private final SparkClosedLoopController leftController;
  private final RelativeEncoder leftEncoder;

  private final SparkFlex rightMotor;
  private final SparkFlexConfig rightConfig;

  SimpleMotorFeedforward shooterFFLeft =
      new SimpleMotorFeedforward(GAINS.kS(), GAINS.kV(), GAINS.kA());
  SimpleMotorFeedforward shooterFFRight =
      new SimpleMotorFeedforward(GAINS.kS(), GAINS.kV(), GAINS.kA());

  private boolean shooterIsEnabled = false;
  private AngularVelocity desiredRPM = RPM.of(0);

  public AnotherShooterIOSparkFlex() {
    // Setup configuration for the left motor
    leftConfig = new SparkFlexConfig();
    leftConfig
        .inverted(false)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(AnotherShooterConstants.MAX_CURRENT);

    leftConfig
        .encoder
        .positionConversionFactor(1 / FLYWHEEL_CONFIG.REDUCTION())
        .velocityConversionFactor(1 / FLYWHEEL_CONFIG.REDUCTION())
        .quadratureMeasurementPeriod(10)
        .quadratureAverageDepth(6);

    leftConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control.
        .pid(
            AnotherShooterConstants.GAINS.kP(),
            AnotherShooterConstants.GAINS.kI(),
            AnotherShooterConstants.GAINS.kD(),
            ClosedLoopSlot.kSlot0)
        .outputRange(-1, 1);

    leftMotor = new SparkFlex(FLYWHEEL_CONFIG.CANID_LEFT(), MotorType.kBrushless);
    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftController = leftMotor.getClosedLoopController();
    leftEncoder = leftMotor.getEncoder();
    leftController.setSetpoint(0, ControlType.kDutyCycle);

    // Make the right motor follow the left motor, but inverted.
    // This way we only have to do closed loop control on one motor, and the other motor will mirror
    // it.
    rightConfig = new SparkFlexConfig();
    rightConfig
        .encoder
        .positionConversionFactor(1 / FLYWHEEL_CONFIG.REDUCTION())
        .velocityConversionFactor(1 / FLYWHEEL_CONFIG.REDUCTION())
        .quadratureMeasurementPeriod(10)
        .quadratureAverageDepth(6);

    rightConfig.follow(leftMotor, true);
    rightMotor = new SparkFlex(FLYWHEEL_CONFIG.CANID_RIGHT(), MotorType.kBrushless);
    rightMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(AnotherShooterIOInputs inputs) {

    inputs.isEnabled = shooterIsEnabled;
    inputs.isAtGoal = leftController.isAtSetpoint();

    inputs.desiredRPM = desiredRPM.in(RPM);

    inputs.currentVelocityRPM = leftEncoder.getVelocity() * FLYWHEEL_CONFIG.REDUCTION();

    inputs.velocityLeft = RPM.of(leftEncoder.getVelocity());
    inputs.velocityRight = RPM.of(rightMotor.getEncoder().getVelocity());

    inputs.ampsLeft = Amps.of(leftMotor.getOutputCurrent());
    inputs.ampsRight = Amps.of(rightMotor.getOutputCurrent());

    inputs.tempLeft = Celsius.of(leftMotor.getMotorTemperature());
    inputs.tempRight = Celsius.of(rightMotor.getMotorTemperature());

    inputs.appliedOutputLeft = leftMotor.getAppliedOutput();
    inputs.appliedOutputRight = rightMotor.getAppliedOutput();
  }

  @Override
  public void start(AngularVelocity new_RPM) {
    desiredRPM = new_RPM;
    leftController.setSetpoint(new_RPM.in(RPM), ControlType.kVelocity, ClosedLoopSlot.kSlot0);

    shooterIsEnabled = true;
  }

  @Override
  public void stop() {
    leftController.setSetpoint(0, ControlType.kDutyCycle);
    shooterIsEnabled = false;
  }

  @Override
  public void setVelocity(AngularVelocity new_RPM) {
    desiredRPM = new_RPM;
    if (shooterIsEnabled) {
      leftController.setSetpoint(new_RPM.in(RPM), ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }
  }

  @Override
  public void startBySetpoint(double new_Setpoint) {
    leftController.setSetpoint(new_Setpoint, ControlType.kDutyCycle);
    shooterIsEnabled = true;
  }

  @Override
  public void setPID(double newkP, double newkI, double newkD) {
    leftConfig.closedLoop.pid(newkP, newkI, newkD, ClosedLoopSlot.kSlot0);
    System.out.printf("New PID: %f, %f, %f", newkP, newkI, newkD);
    leftMotor.configure(
        leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setPID(double newkP, double newkI, double newkD, double newkV, double newkS) {
    leftConfig.closedLoop.pid(newkP, newkI, newkD, ClosedLoopSlot.kSlot0);
    leftConfig.closedLoop.feedForward.sv(newkS, newkV, ClosedLoopSlot.kSlot0);
    System.out.printf("New PID: %f, %f, %f", newkP, newkI, newkD);
    leftMotor.configure(
        leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void runCharacterization(double input) {
    leftMotor.setVoltage(input);
  }

  @Override
  public void setVoltage(double voltage) {
    leftMotor.setVoltage(voltage);
  }
}
