package frc.robot.subsystems.anotherShooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.subsystems.anotherShooter.AnotherShooterConstants.GAINS;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface AnotherShooterIO {

  @AutoLog
  public static class AnotherShooterIOInputs {

    public boolean connected = false;

    public boolean isEnabled = false;

    public AngularVelocity currentVelocity = RPM.of(0);
    public AngularVelocity velocityLeft = RPM.of(0);
    public AngularVelocity velocityRight = RPM.of(0);

    public Current ampsLeft = Amps.of(0);
    public Current ampsRight = Amps.of(0);

    public Temperature tempLeft = Celsius.of(0);
    public Temperature tempRight = Celsius.of(0);

    public double appliedOutputLeft = 0;
    public double appliedOutputRight = 0;

    public AngularVelocity desiredRPM = RPM.of(0);

    public double kP = GAINS.kP();
    public double kI = GAINS.kI();
    public double kD = GAINS.kD();
    public double kS = GAINS.kS();
    public double kV = GAINS.kV();

    public static double oomf = 0.0;
  }

  public default void start(AngularVelocity new_RPM) {}
  ;

  public default void setVelocity(AngularVelocity new_RPM) {}
  ;

  public default void stop() {}
  ;

  public default void startBySetpoint(double new_Setpoint) {}
  ;

  public default void runCharacterization(double input) {}

  public default void setPID(double kP, double kI, double kD) {}

  public default void setPID(double kP, double kI, double kD, double kV, double kS) {}

  public default void updateInputs(AnotherShooterIOInputs inputs) {}

  public default void setVoltage(double voltage) {}
}
