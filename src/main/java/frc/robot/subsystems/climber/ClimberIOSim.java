package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Voltage;

public class ClimberIOSim implements ClimberIO {
  private final double maxPosRad =
      Math.max(
          ClimberConstants.EXTEND_POSITION_LEFT.in(Radians),
          ClimberConstants.EXTEND_POSITION_RIGHT.in(Radians));
  private final double maxSpeedTravelTime = 0.5;
  private final double maxSpeedRadPerSec = maxPosRad / maxSpeedTravelTime;
  private final double kV = maxSpeedRadPerSec / 12; // rad/s/V
  private double position = 0;
  private double velocity = 0;
  private double voltage = 0;

  public ClimberIOSim() {}

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    position += velocity * 0.02;
    if (position > maxPosRad || position < 0) {
      velocity = 0;
      position = MathUtil.clamp(position, 0, maxPosRad);
      inputs.leftCurrent = Amps.of(60);
      inputs.rightCurrent = Amps.of(60);
    } else {
      inputs.leftCurrent = Amps.of(0);
      inputs.rightCurrent = Amps.of(0);
    }

    inputs.leftPosition = Radians.of(position);
    inputs.leftVelocity = RadiansPerSecond.of(velocity);
    inputs.leftAppliedVoltage = Volts.of(voltage);

    inputs.rightPosition = Radians.of(position);
    inputs.rightVelocity = RadiansPerSecond.of(velocity);
    inputs.rightAppliedVoltage = Volts.of(voltage);
    inputs.averagePosition = Radians.of(position);
  }

  @Override
  public void setVoltage(Voltage out) {
    voltage = out.in(Volts);
    velocity = voltage * kV;
  }

  @Override
  public void setLeftVoltage(Voltage out) {
    setVoltage(out);
  }

  @Override
  public void setRightVoltage(Voltage out) {
    setVoltage(out);
  }

  @Override
  public void stop() {
    setVoltage(Volts.of(0));
  }

  // @Override
  // public void stopFront() {
  //     stop();
  // }

  // @Override
  // public void stopBack() {
  //     stop();
  // }
}
