package frc.robot.subsystems.anotherShooter;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.subsystems.anotherShooter.AnotherShooterConstants.GAINS;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface AnotherShooterIO {

  @AutoLog
  public static class AnotherShooterIOInputs {

    public boolean connected = false;

    public boolean isEnabled = false;
    public boolean isAtGoal = false;

    public double velocityRPMFlyWheel = 0;

    public double positionRotationsLeft = 0;
    public double positionRotationsRight = 0;

    public double velocityRPMLeft = 0;
    public double velocityRPMRight = 0;

    public Current ampsLeft = Amps.of(0);
    public Current ampsRight = Amps.of(0);

    public double tempCelsiusLeft = 0;
    public double tempCelsiusRight = 0;

    public double appliedVoltageLeft = 0;
    public double appliedVoltageRight = 0;

    public double velocityRPMDesired = 0;

    public double kP = GAINS.kP();
    public double kI = GAINS.kI();
    public double kD = GAINS.kD();
    public double kS = GAINS.kS();
    public double kV = GAINS.kV();
  }

  public default void start(AngularVelocity new_RPM) {}

  public default void setVelocity(AngularVelocity new_RPM) {}

  public default void stop() {}

  public default void startBySetpoint(double new_Setpoint) {}

  public default void setPID(double kP, double kI, double kD) {}

  public default void setPID(double kP, double kI, double kD, double kV, double kS) {}

  public default void updateInputs(AnotherShooterIOInputs inputs) {}

  public default void setVoltage(double voltage) {}
}
