package frc.robot.subsystems.anotherShooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.anotherShooter.AnotherShooterConstants.FLYWHEEL_CONFIG;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class AnotherShooterIOSim implements AnotherShooterIO {
  private boolean isEnabled = false;
  private boolean connected = true;

  private FlywheelSim leftSim;
  private FlywheelSim rightSim;

  private AngularVelocity desiredRPM = RPM.of(0);

  public AnotherShooterIOSim() {

    leftSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNeoVortex(1), 4 * FLYWHEEL_CONFIG.MOI(), FLYWHEEL_CONFIG.REDUCTION()),
            DCMotor.getNeoVortex(1),
            0.00363458292);
    rightSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNeoVortex(1), 4 * FLYWHEEL_CONFIG.MOI(), FLYWHEEL_CONFIG.REDUCTION()),
            DCMotor.getNeoVortex(1),
            0.00363458292);
  }

  @Override
  public void updateInputs(AnotherShooterIOInputs inputs) {
    rightSim.update(0.02);
    leftSim.update(0.02);

    inputs.isEnabled = isEnabled;
    inputs.connected = connected;

    inputs.velocityRPMDesired = desiredRPM.in(RPM);
    inputs.velocityRPMFlyWheel = leftSim.getAngularVelocityRPM() / FLYWHEEL_CONFIG.REDUCTION();

    inputs.velocityRPMLeft = leftSim.getAngularVelocityRPM();
    inputs.velocityRPMRight = rightSim.getAngularVelocityRPM();

    inputs.ampsLeft = Amps.of(leftSim.getCurrentDrawAmps());
    inputs.ampsRight = Amps.of(rightSim.getCurrentDrawAmps());

    inputs.appliedVoltageLeft = leftSim.getInputVoltage();
    inputs.appliedVoltageRight = rightSim.getInputVoltage();
  }

  @Override
  public void start(AngularVelocity new_RPM) {
    desiredRPM = new_RPM;

    rightSim.setAngularVelocity(new_RPM.in(RadiansPerSecond));
    leftSim.setAngularVelocity(new_RPM.in(RadiansPerSecond));
    isEnabled = true;
  }

  @Override
  public void stop() {
    desiredRPM = RPM.of(0);
    setVelocity(desiredRPM);
    isEnabled = false;
  }

  @Override
  public void setVelocity(AngularVelocity new_RPM) {
    if (isEnabled) {
      rightSim.setAngularVelocity(new_RPM.in(RotationsPerSecond));
      leftSim.setAngularVelocity(new_RPM.in(RotationsPerSecond));
    }
  }

  @Override
  public void startBySetpoint(double new_Setpoint) {
    isEnabled = true;
    System.out.printf("Motors Setpoint: %f", new_Setpoint);
  }

  @Override
  public void setPID(double newkP, double newkI, double newkD) {
    System.out.printf("New PID: %f, %f, %f \n", newkP, newkI, newkD);
  }

  @Override
  public void setVoltage(double voltage) {}
}
