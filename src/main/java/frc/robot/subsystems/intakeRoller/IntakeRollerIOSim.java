// Copyright 2025-2026 FRC 8626
// https://github.com/team8626
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

package frc.robot.subsystems.intakeRoller;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intakeRoller.IntakeRollerConstants.ROLLER_CONFIG;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeRollerIOSim implements IntakeRollerIO {
  private boolean isEnabled = false;
  private boolean connected = true;

  private final FlywheelSim motorSim;

  /** Target velocity for closed-loop control. */
  private AngularVelocity desiredWheelVelocity = RPM.of(0.0);

  public IntakeRollerIOSim() {
    motorSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNeoVortex(1), 4 * ROLLER_CONFIG.MOI(), ROLLER_CONFIG.REDUCTION()),
            DCMotor.getNeoVortex(1),
            0.00363458292);
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    motorSim.update(0.02);

    inputs.isEnabled = isEnabled;
    inputs.connected = connected;

    inputs.velocityRPMDesired = desiredWheelVelocity.in(RPM);
    inputs.velocityRPMRollers = motorSim.getAngularVelocityRPM() / ROLLER_CONFIG.REDUCTION();
    inputs.velocityRPMMotor = motorSim.getAngularVelocityRPM();

    inputs.current = Amps.of(Math.abs(motorSim.getCurrentDrawAmps()));
    inputs.appliedVoltage = Volts.of(motorSim.getInputVoltage());
  }

  @Override
  public void start(AngularVelocity new_velocity) {
    desiredWheelVelocity = new_velocity;
    setVelocity(desiredWheelVelocity);
    isEnabled = true;
  }

  @Override
  public void stop() {
    desiredWheelVelocity = RPM.of(0.0);
    setVelocity(desiredWheelVelocity);
    isEnabled = false;
  }

  private void setVelocity(AngularVelocity new_RPM) {
    motorSim.setAngularVelocity(new_RPM.times(ROLLER_CONFIG.REDUCTION()).in(RadiansPerSecond));
  }

  @Override
  public void setPID(double kP, double kI, double kD, double kV, double kS) {
    System.out.printf("New PID: %f, %f, %f \n", kP, kI, kD);
  }

  @Override
  public void setVoltage(double voltage) {}

  // Use for unit testing purpose only
  @Deprecated
  public void disconnect() {
    connected = false;
  }
}
