package frc.robot.subsystems.anotherShooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;

public class AnotherShooterConstants {

  // Tuned Values
  public static final int MAX_CURRENT = 50; // Amps

  public static final AngularVelocity VELOCITY_TOLERANCE = RPM.of(25);

  public static final Angle ANOTHERSHOOTER_ANGLE = Degrees.of(55);
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
        case REBUILT_COMPBOT -> new FlywheelConfig(7, 6, 1.0 / 1.0, MOMENT_OF_INERTIA, 6000.0);
          // All other case use the same as simbot for now, but this should be changed when we have
          // another real to test on
        default -> new FlywheelConfig(0, 0, (1 / 1), MOMENT_OF_INERTIA, 6000.0);
      };

  // PID Constants
  public static final Gains GAINS =
      switch (Constants.robot) {
        case REBUILT_COMPBOT -> new Gains(0.001, 0.0, 0.0006, 0.10395, 0.0019, 0);
          // All other case use the same as simbot for now, but this should be changed when we have
          // another real to test on
        default -> new Gains(0.05, 0.0, 0.0, 0.10395, 0.00296, 0.0);
      };

  public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}

  public record FlywheelConfig(
      int CANID_LEFT,
      int CANID_RIGHT,
      double REDUCTION,
      double MOI,
      double MAX_ACCELERATION_RPM_PER_SEC) {}
}
