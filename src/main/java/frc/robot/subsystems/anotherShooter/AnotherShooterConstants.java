package frc.robot.subsystems.anotherShooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;

public class AnotherShooterConstants {

  // Tuned Values
  public static final int MAX_CURRENT = 50; // Amps

  public static final AngularVelocity DEFAULT_VELOCITY = RPM.of(2500);
  public static final AngularVelocity VELOCITY_TOLERANCE = RPM.of(25);
  public static final AngularVelocity UNJAM_VELOCITY = RPM.of(-500);

  public static final double DEFAULT_SHOT_EFFICIENCY = 1.00;

  public static final Time STOP_DELAY = Milliseconds.of(200);

  public static final Angle ANOTHERSHOOTER_ANGLE = Degrees.of(67);
  public static final Transform3d ANOTHERSHOOTER_OFFSET =
      new Transform3d(
          Inches.of(-8),
          Inches.of(4.5),
          Inches.of(16.25),
          new Rotation3d(new Rotation2d(Degrees.of(0))));

  // AlgaeShooter Constants
  public static final Distance FLYWHEEL_RADIUS = Inches.of(2);
  public static final double FLYWHEEL_MASS = Units.lbsToKilograms(3.76);
  // private static final double stealthWheelMomentOfInertia = 0.5 * wheelMassKg * wheelRadiusMeters
  // * wheelRadiusMeters;
  private static final double MOMENT_OF_INERTIA = 1;

  // Flywheel Config
  public static final FlywheelConfig FLYWHEEL_CONFIG =
      switch (Constants.robot) {
        case REBUILT_PHOENIX, REBUILT_AKIT -> new FlywheelConfig(
            6, 7, 1.0 / 1.0, MOMENT_OF_INERTIA, 6000.0);
          // All other case use the same as simbot for now, but this should be changed when we have
          // another real to test on
        default -> new FlywheelConfig(6, 7, (1 / 1), MOMENT_OF_INERTIA, 6000.0);
      };

  // PID Constants
  public static final Gains GAINS =
      switch (Constants.robot) {
        case REBUILT_PHOENIX, REBUILT_AKIT -> new Gains(
            // Characterizarion Runs
            // kp= <NaN>,     ks= 0.079702, kv= 0.0018007, ka= 0.00014799)
            // kp= 1.6088e-7, ks= 0.099766, kv= 0.0017934, ka= 0.00014058
            // kp= 1.0364e-6, ks= 0.10757,  kv= 0.0017894, ka= 0.00014797

            0.0005, 0.0, 0.00015, 0.0957, 0.00179, 0.000146);
          // All other case use the same as simbot for now, but this should be changed when we have
          // another real to test on
        default -> new Gains(0.0, 0.0, 0.0, 0, 0, 0.0);
      };

  public static final AngularVelocity MAX_VELOCITY = RPM.of(6700 / FLYWHEEL_CONFIG.REDUCTION());

  public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}

  public record FlywheelConfig(
      int CANID_LEFT,
      int CANID_RIGHT,
      double REDUCTION,
      double MOI,
      double MAX_ACCELERATION_RPM_PER_SEC) {}

  public static final InterpolatingDoubleTreeMap RPMMap = new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap IndexerMap = new InterpolatingDoubleTreeMap();

  static {
    // format is (distance to target in feet, required shooter velocity in RPM)
    RPMMap.put(1.0, 1500.0);
    RPMMap.put(4.68, 2150.0);
    RPMMap.put(6.85, 2400.0);
    RPMMap.put(8.63, 2575.0);
    RPMMap.put(9.92, 2700.0);
    RPMMap.put(11.20, 2870.0);
    RPMMap.put(17.33, 4100.0);
  }

  static {
    // format is (distance to target in feet, required shooter velocity in RPM)
    IndexerMap.put(1.0, 42.0);
    IndexerMap.put(4.68, 42.0);
    IndexerMap.put(6.85, 42.0);
    IndexerMap.put(8.63, 42.0);
    IndexerMap.put(9.92, 42.0);
    IndexerMap.put(11.20, 15.0);
    IndexerMap.put(17.33, 12.0);
  }
}
