package frc.robot.subsystems.anotherShooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.anotherShooter.AnotherShooterConstants.FLYWHEEL_CONFIG;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class AnotherShooterIOSim implements AnotherShooterIO {

  private boolean isEnabled = false;

  private FlywheelSim leftSim;
  private FlywheelSim rightSim;

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

    inputs.currentVelocity = RPM.of(leftSim.getAngularVelocityRPM() * FLYWHEEL_CONFIG.REDUCTION());

    inputs.velocityLeft = RPM.of(leftSim.getAngularVelocityRPM());
    inputs.velocityRight = RPM.of(rightSim.getAngularVelocityRPM());

    inputs.ampsLeft = Amps.of(leftSim.getCurrentDrawAmps());
    inputs.ampsRight = Amps.of(rightSim.getCurrentDrawAmps());

    inputs.appliedOutputLeft =
        leftSim.getInputVoltage() / 12.0; // Convert voltage to a percentage for applied output
    inputs.appliedOutputRight =
        rightSim.getInputVoltage() / 12.0; // Convert voltage to a percentage for applied output
  }

  // @Override
  // public void updateInputs(
  //     AlgaeShooterValues values) { // what is going on here does this need to be Algae
  //   rightSim.update(0.02);
  //   leftSim.update(0.02);
  //   launchSim.update(0.02);

  //   values.launchIsEnabled = launcherIsEnabled;
  //   values.shooterIsEnabled = shooterIsEnabled;

  //   values.currentRPMLeft = getShooterRPMLeft();
  //   values.currentRPMRight = getShooterRPMRight();

  //   values.currentRMPLauncher = getLauncherRPM();
  //   values.currentLauncherSetpoint = getLauncherSetpoint();

  //   values.ampsLeft = Amps.of(leftSim.getCurrentDrawAmps());
  //   values.ampsRight = Amps.of(rightSim.getCurrentDrawAmps());
  //   values.ampsLauncher = Amps.of(launchSim.getCurrentDrawAmps());

  //   values.isLoaded = shooterIsLoaded();
  // }

  @Override
  public void start(AngularVelocity new_RPM) {
    rightSim.setAngularVelocity(new_RPM.in(RadiansPerSecond));
    leftSim.setAngularVelocity(new_RPM.in(RadiansPerSecond));
    isEnabled = true;
    System.out.printf("Shooter RPM: %f", new_RPM.in(RPM));
  }

  @Override
  public void stop() {
    setVelocity(RPM.of(0));
    isEnabled = false;
  }

  @Override
  public void setVelocity(AngularVelocity new_RPM) {
    if (isEnabled) {
      rightSim.setAngularVelocity(new_RPM.in(RotationsPerSecond));
      leftSim.setAngularVelocity(new_RPM.in(RotationsPerSecond));
      System.out.printf("Shooter RPM: %f", new_RPM.in(RPM));
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
