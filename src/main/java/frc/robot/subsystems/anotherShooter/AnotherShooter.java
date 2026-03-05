package frc.robot.subsystems.anotherShooter;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class AnotherShooter extends SubsystemBase {
  private AnotherShooterIO algaeShooterInterface;
  private final AnotherShooterIO io;
  private final AnotherShooterIOInputsAutoLogged inputs = new AnotherShooterIOInputsAutoLogged();

  private AngularVelocity DesiredRPM;

  /** Shown on the dashboard when the index motor is not connected. */
  private final Alert motorDisconnectedAlert =
      new Alert("AnotherShooter motor disconnected.", AlertType.kError);

  private final LoggedTunableNumber flywheelKP =
      new LoggedTunableNumber("AnotherShooter/Flywheel/kP", AnotherShooterConstants.GAINS.kP());
  private final LoggedTunableNumber flywheelKI =
      new LoggedTunableNumber("AnotherShooter/Flywheel/kI", AnotherShooterConstants.GAINS.kI());
  private final LoggedTunableNumber flywheelKD =
      new LoggedTunableNumber("AnotherShooter/Flywheel/kD", AnotherShooterConstants.GAINS.kD());
  private final LoggedTunableNumber flywheelKV =
      new LoggedTunableNumber("AnotherShooter/Flywheel/kV", AnotherShooterConstants.GAINS.kV());
  private final LoggedTunableNumber flywheelKS =
      new LoggedTunableNumber("AnotherShooter/Flywheel/kS", AnotherShooterConstants.GAINS.kS());

  public AnotherShooter(AnotherShooterIO io) {
    this.io = io;
  }

  public void start(AngularVelocity newRPM) {
    io.setVelocity(newRPM);
    io.start(DesiredRPM);
  }

  public void setVelocity(AngularVelocity newRPM) {
    io.setVelocity(newRPM);
  }

  public void startBySetpoint(double new_Setpoint) {
    io.startBySetpoint(new_Setpoint);
  }

  public void stop() {
    io.stop();
  }

  public AngularVelocity getVelocity() {
    return inputs.currentVelocity;
  }

  public void setPID(double newkP, double newkI, double newkD) {
    algaeShooterInterface.setPID(newkP, newkI, newkD);
  }

  public void setkP(double newkP) {
    algaeShooterInterface.setPID(newkP, inputs.kI, inputs.kD);
  }

  public void setkI(double newkI) {
    algaeShooterInterface.setPID(inputs.kP, newkI, inputs.kD);
  }

  public void setkD(double newkD) {
    algaeShooterInterface.setPID(inputs.kP, inputs.kI, newkD);
  }

  @Override
  public void periodic() {
    updateTunables();
    io.updateInputs(inputs);
    Logger.processInputs("AnotherShooter", inputs);

    // Update alerts
    motorDisconnectedAlert.set(!inputs.connected);
  }

  private void updateTunables() {
    if (flywheelKP.hasChanged(hashCode())
        || flywheelKI.hasChanged(hashCode())
        || flywheelKD.hasChanged(hashCode())
        || flywheelKV.hasChanged(hashCode())
        || flywheelKS.hasChanged(hashCode())) {
      io.setPID(
          flywheelKP.get(), flywheelKI.get(), flywheelKD.get(), flywheelKV.get(), flywheelKS.get());
    }
  }

  // Characterization methods
  public void runCharacterization(double input) {
    io.runCharacterization(input);
  }

  public double getCharacterizationVelocity() {
    return inputs.currentVelocity.in(RPM);
  }
}
