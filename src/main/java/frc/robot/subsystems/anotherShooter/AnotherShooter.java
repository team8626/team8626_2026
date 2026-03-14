package frc.robot.subsystems.anotherShooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class AnotherShooter extends SubsystemBase {
  private AnotherShooterIO anotherShooterInterface;
  private final AnotherShooterIO io;
  private final AnotherShooterIOInputsAutoLogged inputs = new AnotherShooterIOInputsAutoLogged();

  /** Shown on the dashboard when the index motor is not connected. */
  private final Alert motorDisconnectedAlert =
      new Alert("AnotherShooter motor disconnected.", AlertType.kError);

  // Configure SysId
  private SysIdRoutine sysId =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              null,
              null,
              (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
          new SysIdRoutine.Mechanism(
              (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

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
  private final LoggedTunableNumber flywheelRPM =
      new LoggedTunableNumber(
          "AnotherShooter/Flywheel/RPM", AnotherShooterConstants.DEFAULT_VELOCITY.in(RPM));

  public AnotherShooter(AnotherShooterIO io) {
    this.io = io;
  }

  public void start(AngularVelocity new_velocity) {
    AngularVelocity velocity = new_velocity;

    if ((new_velocity.abs(RPM)) > AnotherShooterConstants.MAX_VELOCITY.in(RPM)) {
      velocity = RPM.of(AnotherShooterConstants.MAX_VELOCITY.copySign(velocity, RPM));
    }
    io.start(new_velocity);
  }

  public void start() {
    io.start(RPM.of(flywheelRPM.get()));
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

  public void setPID(double new_kP, double new_kI, double new_kD) {
    anotherShooterInterface.setPID(new_kP, new_kI, new_kD);
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

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  public void setVoltage(double voltage) {
    anotherShooterInterface.setVoltage(voltage);
  }
}
