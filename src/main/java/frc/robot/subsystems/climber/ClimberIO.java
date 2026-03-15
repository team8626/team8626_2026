package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public boolean leftConnected = false;
    public Angle leftPosition = Radians.of(0);
    public AngularVelocity leftVelocity = RadiansPerSecond.of(0);
    public Current leftCurrent = Amps.of(0);
    public Voltage leftAppliedVoltage = Volts.of(0);

    public boolean rightConnected = false;
    public Angle rightPosition = Radians.of(0);
    public AngularVelocity rightVelocity = RadiansPerSecond.of(0);
    public Current rightCurrent = Amps.of(0);
    public Voltage rightAppliedVoltage = Volts.of(0);

    public Angle averagePosition = Radians.of(0);
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setVoltage(Voltage out) {}

  public default void setLeftVoltage(Voltage out) {}

  public default void setRightVoltage(Voltage out) {}

  public default void stop() {}

  public default void stopLeft() {}

  public default void stopRight() {}

  public default void zeroPosition() {}

  public default void setNeutralMode(NeutralModeValue neutralModeValue) {}
}
