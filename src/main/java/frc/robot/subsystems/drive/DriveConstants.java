// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
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

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.util.TunableControls.ControlConstants;
import frc.robot.util.TunableControls.TunableControlConstants;
import java.util.function.Supplier;

public class DriveConstants {
  //   public static final double maxSpeedMetersPerSec = 4.8;

  public static final LinearVelocity DEFAULT_DRIVE_SPEED = MetersPerSecond.of(3.2);
  public static final AngularVelocity DEFAULT_ROT_SPEED = RotationsPerSecond.of(0.75);

  public static final LinearVelocity FAST_DRIVE_SPEED = MetersPerSecond.of(4.5);
  public static final AngularVelocity FAST_ROT_SPEED = RotationsPerSecond.of(2);

  public static final LinearAcceleration MAX_TELEOP_ACCEL = MetersPerSecondPerSecond.of(25);

  public static final Frequency ODOMETRY_UPDATE_FREQ;
  public static final Distance MODULE_DISTANCE_Y; // left to right
  public static final Distance MODULE_DISTANCE_X; // front to back

  // Drive motor configuration
  public static final Current DRIVE_MOTOR_CURRENT_LIMIT;
  public static final double DRIVE_GEAR_RATIO;
  public static final DCMotor DRIVE_GEARBOX;
  public static final LinearVelocity SPEED_AT_12V; // theoretical free speed
  public static final Distance WHEEL_RADIUS;
  private static final double WHEEL_COF; // coefficient of friction for wheel-ground interaction

  public static final double STEER_GEAR_RATIO;
  public static final DCMotor STEER_GEARBOX;

  // Turn Motor configuration
  public static final Current TURN_MOTOR_CURRENT_LIMIT;

  static {
    switch (Constants.robot) {
      case REBUILT_COMPBOT, SIMBOT -> {
        ODOMETRY_UPDATE_FREQ = Hertz.of(250);
        MODULE_DISTANCE_Y = Inches.of(22); // left to right
        MODULE_DISTANCE_X = Inches.of(22); // front to back

        WHEEL_RADIUS = Inches.of(1.985);
        WHEEL_COF = 2.255;
        DRIVE_MOTOR_CURRENT_LIMIT = Amps.of(80);
        DRIVE_GEAR_RATIO = 5.8909; // WCP Swerve x2i 11t pinion, X3 configuration
        DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
        SPEED_AT_12V = MetersPerSecond.of(5.42);

        TURN_MOTOR_CURRENT_LIMIT = Amps.of(40);
        STEER_GEAR_RATIO = 12.1; // WCP Swerve x2
        STEER_GEARBOX = DCMotor.getKrakenX60Foc(1);
      }
      case TSUNAMI -> {
        ODOMETRY_UPDATE_FREQ = Hertz.of(100);
        MODULE_DISTANCE_Y = Inches.of(23.491162); // left to right
        MODULE_DISTANCE_X = Inches.of(23.491162); // front to back

        WHEEL_RADIUS = Inches.of(1.5);
        WHEEL_COF = 1.2;
        DRIVE_MOTOR_CURRENT_LIMIT = Amps.of(50);
        DRIVE_GEAR_RATIO = 4.71; // MAXSwerve in Base kit High configuration
        DRIVE_GEARBOX = DCMotor.getNeoVortex(1);
        SPEED_AT_12V = MetersPerSecond.of(5.74);

        TURN_MOTOR_CURRENT_LIMIT = Amps.of(20);
        STEER_GEAR_RATIO = 9424.0 / 203.0;
        STEER_GEARBOX = DCMotor.getNeo550(1);
      }
      default -> throw new IllegalStateException("Unexpected robot: " + Constants.robot);
    }
  }

  public static final Distance DRIVE_BASE_RADIUS =
      Meters.of(Math.hypot(MODULE_DISTANCE_Y.in(Meters) / 2.0, MODULE_DISTANCE_X.in(Meters) / 2.0));

  public static final Supplier<Translation2d[]> MODULE_TRANSLATIONS =
      () ->
          new Translation2d[] {
            new Translation2d(
                MODULE_DISTANCE_Y.in(Meters) / 2.0, MODULE_DISTANCE_X.in(Meters) / 2.0),
            new Translation2d(
                MODULE_DISTANCE_Y.in(Meters) / 2.0, -MODULE_DISTANCE_X.in(Meters) / 2.0),
            new Translation2d(
                -MODULE_DISTANCE_Y.in(Meters) / 2.0, MODULE_DISTANCE_X.in(Meters) / 2.0),
            new Translation2d(
                -MODULE_DISTANCE_Y.in(Meters) / 2.0, -MODULE_DISTANCE_X.in(Meters) / 2.0)
          };

  // --------------------------------------------------------------------------
  // TSUNAMI Module-specific constants, used in module IO implementations and module construction
  public static class Tsunami_DriveConstants {

    // Zeroed rotation values for each module, see setup instructions
    public static final Rotation2d frontLeftZeroRotation = new Rotation2d(0.0);
    public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0);
    public static final Rotation2d backLeftZeroRotation = new Rotation2d(0.0);
    public static final Rotation2d backRightZeroRotation = new Rotation2d(0.0);

    // Device CAN IDs
    public static final int frontLeftDriveCanId = 15;
    public static final int backLeftDriveCanId = 16;
    public static final int frontRightDriveCanId = 18;
    public static final int backRightDriveCanId = 17;

    public static final int frontLeftTurnCanId = 5;
    public static final int backLeftTurnCanId = 6;
    public static final int frontRightTurnCanId = 8;
    public static final int backRightTurnCanId = 7;

    // Drive encoder configuration
    public static final double driveEncoderPositionFactor =
        2 * Math.PI / DRIVE_GEAR_RATIO; // Rotor Rotations -> Wheel Radians

    public static final double driveEncoderVelocityFactor =
        (2 * Math.PI) / 60.0 / DRIVE_GEAR_RATIO; // Rotor RPM -> Wheel Rad/Sec

    // Drive PID configuration
    public static final double driveKp = 0.0;
    public static final double driveKd = 0.0;
    public static final double driveKs = 0.0;
    public static final double driveKv = 0.1;

    // Turn motor configuration
    public static final boolean turnInverted = false;

    // Turn encoder configuration
    public static final boolean turnEncoderInverted = true;
    public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
    public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

    // Turn PID configuration
    public static final double turnKp = 2.0;
    public static final double turnKd = 0.0;
    public static final double turnPIDMinInput = 0; // Radians
    public static final double turnPIDMaxInput = 2 * Math.PI; // Radians
  }

  // --------------------------------------------------------------------------
  // REBUIL_ROBOT Module-specific constants, used in module IO implementations and module
  // construction
  public static class Rebuilt_SwerveConstants {

    public static final Slot0Configs STEER_GAINS =
        new Slot0Configs()
            .withKP(1000)
            .withKI(0)
            .withKD(8)
            .withKS(6)
            .withKV(0)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final Slot0Configs DRIVE_GAINS =
        new Slot0Configs()
            .withKP(2)
            .withKI(0.0)
            .withKD(0.0)
            .withKS(0.237) // 0.11367, 0.1301, 0.15349, 0.16187 -> 0.140
            .withKV(0.733) // 0.13879, 0.13555, 0.13894, 0.13109 -> 0.136
            .withKA(0.0); // 0.016363, 0.016268, 0.0085342, 0.011084 -> 0.013

    private static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT =
        ClosedLoopOutputType.TorqueCurrentFOC;
    private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT =
        ClosedLoopOutputType.Voltage;

    private static final DriveMotorArrangement DRIVE_MOTOR_TYPE =
        DriveMotorArrangement.TalonFX_Integrated;
    private static final SteerMotorArrangement STEER_MOTOR_TYPE =
        SteerMotorArrangement.TalonFX_Integrated;

    private static final SteerFeedbackType STEER_FEEDBACK_TYPE = SteerFeedbackType.FusedCANcoder;

    private static final Current SLIP_CURRENT = Amps.of(95); // NEEDS TUNING

    private static final TalonFXConfiguration DRIVE_CONFIGS =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true))
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

    private static final TalonFXConfiguration STEER_CONFIGS =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(TURN_MOTOR_CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true))
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withMotionMagic(
                new MotionMagicConfigs().withMotionMagicExpo_kA(0.5).withMotionMagicExpo_kV(2.0));

    private static final CANcoderConfiguration ENCODER_CONFIGS = new CANcoderConfiguration();

    public static final Pigeon2Configuration PIGEON_CONFIGS =
        new Pigeon2Configuration()
            .withMountPose(new MountPoseConfigs().withMountPoseYaw(Degrees.of(-90)));

    // Every 1 rotation of the azimuth results in COUPLE_RATIO drive motor turns;
    private static final double COUPLE_RATIO = 3.375;

    public static final int PIGEON_ID = 6;

    // SIMULATION inertia
    private static final MomentOfInertia STEER_INERTIA = KilogramSquareMeters.of(0.01);
    private static final MomentOfInertia DRIVE_INERTIA = KilogramSquareMeters.of(0.01);
    // SIMULATION voltage necessary to overcome friction
    private static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.2);
    private static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.2);

    public static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS =
        new SwerveDrivetrainConstants()
            .withCANBusName(RobotConstants.CAN_FD_BUS.getName())
            .withPigeon2Id(PIGEON_ID)
            .withPigeon2Configs(PIGEON_CONFIGS);

    private static final SwerveModuleConstantsFactory<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        CONSTANT_CREATOR =
            new SwerveModuleConstantsFactory<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                .withSteerMotorGearRatio(STEER_GEAR_RATIO)
                .withCouplingGearRatio(COUPLE_RATIO)
                .withWheelRadius(WHEEL_RADIUS)
                .withSteerMotorGains(STEER_GAINS)
                .withDriveMotorGains(DRIVE_GAINS)
                .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT)
                .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT)
                .withSlipCurrent(SLIP_CURRENT)
                .withSpeedAt12Volts(SPEED_AT_12V)
                .withDriveMotorType(DRIVE_MOTOR_TYPE)
                .withSteerMotorType(STEER_MOTOR_TYPE)
                .withFeedbackSource(STEER_FEEDBACK_TYPE)
                .withDriveMotorInitialConfigs(DRIVE_CONFIGS)
                .withSteerMotorInitialConfigs(STEER_CONFIGS)
                .withEncoderInitialConfigs(ENCODER_CONFIGS)
                .withSteerInertia(STEER_INERTIA)
                .withDriveInertia(DRIVE_INERTIA)
                .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
                .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE);

    public static class FrontLeft {
      private static final int DRIVE_ID = 11;
      private static final int STEER_ID = 21;
      private static final int ENCODER_ID = 31;
      private static final Angle ENCODER_OFFSET = Rotations.of(0.28564453125);
      private static final boolean STEER_INVERTED = false;
      private static final boolean ENCODER_INVERTED = false;
      private static final boolean DRIVE_INVERTED = false;

      public static final Distance X_POS = MODULE_DISTANCE_X.div(2);
      public static final Distance Y_POS = MODULE_DISTANCE_Y.div(2);

      public static final SwerveModuleConstants<
              TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          MODULE_CONSTANTS =
              CONSTANT_CREATOR.createModuleConstants(
                  STEER_ID,
                  DRIVE_ID,
                  ENCODER_ID,
                  ENCODER_OFFSET,
                  X_POS,
                  Y_POS,
                  DRIVE_INVERTED,
                  STEER_INVERTED,
                  ENCODER_INVERTED);
    }

    public static class FrontRight {
      private static final int DRIVE_ID = 14;
      private static final int STEER_ID = 24;
      private static final int ENCODER_ID = 34;
      private static final Angle ENCODER_OFFSET = Rotations.of(-0.109375);
      private static final boolean STEER_INVERTED = false;
      private static final boolean ENCODER_INVERTED = false;
      private static final boolean DRIVE_INVERTED = true;

      public static final Distance X_POS = MODULE_DISTANCE_X.div(2);
      public static final Distance Y_POS = MODULE_DISTANCE_Y.div(-2);

      public static final SwerveModuleConstants<
              TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          MODULE_CONSTANTS =
              CONSTANT_CREATOR.createModuleConstants(
                  STEER_ID,
                  DRIVE_ID,
                  ENCODER_ID,
                  ENCODER_OFFSET,
                  X_POS,
                  Y_POS,
                  DRIVE_INVERTED,
                  STEER_INVERTED,
                  ENCODER_INVERTED);
    }

    public static class BackLeft {
      private static final int DRIVE_ID = 12;
      private static final int STEER_ID = 22;
      private static final int ENCODER_ID = 32;
      private static final Angle ENCODER_OFFSET = Rotations.of(-0.188232421875);
      private static final boolean STEER_INVERTED = false;
      private static final boolean ENCODER_INVERTED = false;
      private static final boolean DRIVE_INVERTED = false;

      public static final Distance X_POS = MODULE_DISTANCE_X.div(-2);
      public static final Distance Y_POS = MODULE_DISTANCE_Y.div(2);

      public static final SwerveModuleConstants<
              TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          MODULE_CONSTANTS =
              CONSTANT_CREATOR.createModuleConstants(
                  STEER_ID,
                  DRIVE_ID,
                  ENCODER_ID,
                  ENCODER_OFFSET,
                  X_POS,
                  Y_POS,
                  DRIVE_INVERTED,
                  STEER_INVERTED,
                  ENCODER_INVERTED);
    }

    public static class BackRight {
      private static final int DRIVE_ID = 13;
      private static final int STEER_ID = 23;
      private static final int ENCODER_ID = 33;
      private static final Angle ENCODER_OFFSET = Rotations.of(-0.134033203125);
      private static final boolean STEER_INVERTED = false;
      private static final boolean ENCODER_INVERTED = false;
      private static final boolean DRIVE_INVERTED = true;

      public static final Distance X_POS = MODULE_DISTANCE_X.div(-2);
      public static final Distance Y_POS = MODULE_DISTANCE_Y.div(-2);

      public static final SwerveModuleConstants<
              TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          MODULE_CONSTANTS =
              CONSTANT_CREATOR.createModuleConstants(
                  STEER_ID,
                  DRIVE_ID,
                  ENCODER_ID,
                  ENCODER_OFFSET,
                  X_POS,
                  Y_POS,
                  DRIVE_INVERTED,
                  STEER_INVERTED,
                  ENCODER_INVERTED);
    }

    // public static final Supplier<Translation2d[]> GET_MODULE_POSITIONS =
    //     () ->
    //         new Translation2d[] {
    //           new Translation2d(
    //               Rebuilt_SwerveConstants.FrontLeft.X_POS,
    // Rebuilt_SwerveConstants.FrontLeft.Y_POS),
    //           new Translation2d(
    //               Rebuilt_SwerveConstants.FrontRight.X_POS,
    //               Rebuilt_SwerveConstants.FrontRight.Y_POS),
    //           new Translation2d(
    //               Rebuilt_SwerveConstants.BackLeft.X_POS,
    // Rebuilt_SwerveConstants.BackLeft.Y_POS),
    //           new Translation2d(
    //               Rebuilt_SwerveConstants.BackRight.X_POS,
    // Rebuilt_SwerveConstants.BackRight.Y_POS),
    //         };

    public static final Matrix<N3, N1> ODOMETRY_STD_DEV = VecBuilder.fill(0.02, 0.02, 0.01);

    public static final DriveRequestType DRIVE_REQUEST_TYPE = DriveRequestType.Velocity;
    public static final SteerRequestType STEER_REQUEST_TYPE = SteerRequestType.MotionMagicExpo;

    public static final LinearVelocity LINEAR_VEL_DEADBAND = MetersPerSecond.of(0.02);
    public static final AngularVelocity ANGLULAR_VEL_DEADBAND = DegreesPerSecond.of(1);

    // Characterization
    public static final Time FF_START_DELAY = Seconds.of(2.0);
    public static final Velocity<VoltageUnit> FF_RAMP_RATE = Volts.of(0.1).per(Second);
    public static final AngularVelocity FF_WHEEL_RADIUS_MAX_VELOCITY = RadiansPerSecond.of(0.25);
    public static final AngularAcceleration FF_WHEEL_RADIUS_RAMP_RATE =
        RadiansPerSecondPerSecond.of(0.05);
  }

  public static final class AutoConstants {
    // --------------------------------------------------------------------------
    // PathPlanner configuration
    public static final double robotMassKg = 74.088;
    public static final double robotMOI = 6.883;
    public static final RobotConfig PP_CONFIG =
        new RobotConfig(
            robotMassKg,
            robotMOI,
            new ModuleConfig(
                WHEEL_RADIUS,
                SPEED_AT_12V,
                WHEEL_COF,
                DRIVE_GEARBOX,
                DRIVE_GEAR_RATIO,
                DRIVE_MOTOR_CURRENT_LIMIT,
                1),
            MODULE_TRANSLATIONS.get());
  }

  // --------------------------------------------------------------------------
  // Alignment
  private static final ControlConstants TRENCH_TRANSLATION_BASE_CONSTANTS =
      new ControlConstants().withPID(6, 0, 0);
  private static final ControlConstants ROTATION_BASE_CONSTANTS =
      new ControlConstants().withPID(8, 0, 0).withContinuous(-180, 180);

  public static final TunableControlConstants TRENCH_TRANSLATION_CONSTANTS =
      new TunableControlConstants("Swerve/Trench Translation", TRENCH_TRANSLATION_BASE_CONSTANTS);
  public static final TunableControlConstants ROTATION_CONSTANTS =
      new TunableControlConstants("Swerve/Rotation", ROTATION_BASE_CONSTANTS);
}
