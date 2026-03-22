package frc.robot.subsystems.anotherShooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class AnotherShooter extends SubsystemBase {
  private final AnotherShooterIO io;
  private final AnotherShooterIOInputsAutoLogged inputs = new AnotherShooterIOInputsAutoLogged();

  private double lastShotTimestampSec = -999.0;
  private boolean feeding = false;
  private boolean shotInProgress = false;

  private static final double SHOT_DEBOUNCE_SEC = 0.05;
  private static final double SHOT_RPM_DROP_THRESHOLD = 120.0;
  private static final double SHOT_CURRENT_THRESHOLD_AMPS = 20.0;
  private static final double MIN_TARGET_RPM = 1500.0;

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
          new SysIdRoutine.Mechanism((voltage) -> setVoltage(voltage.in(Volts)), null, this));

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
    return RPM.of(inputs.velocityRPMFlyWheel);
  }

  public void setPID(double new_kP, double new_kI, double new_kD) {
    io.setPID(new_kP, new_kI, new_kD);
  }

  public boolean isAtGoal() {
    return inputs.isAtGoal;
  }

  @Override
  public void periodic() {
    updateTunables();
    io.updateInputs(inputs);
    Logger.processInputs("AnotherShooter", inputs);

    // Shot counting logic
    updateShotEstimate();

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

  public void startFeeding() {
    feeding = true;
  }

  public void stopFeeding() {
    feeding = false;
  }

  public void resetShotCount() {
    inputs.shotCount = 0;
  }

  private void updateShotEstimate() {
    double now = Timer.getFPGATimestamp();

    double targetRpm = inputs.velocityRPMDesired;
    double actualRpm = inputs.velocityRPMFlyWheel;
    double rpmDrop = targetRpm - actualRpm;

    double avgCurrentAmps = (inputs.ampsLeft.in(Amps) + inputs.ampsRight.in(Amps)) / 2.0;

    boolean shotCondition =
        inputs.isEnabled
            && feeding
            && targetRpm > MIN_TARGET_RPM
            && rpmDrop > SHOT_RPM_DROP_THRESHOLD
            && avgCurrentAmps > SHOT_CURRENT_THRESHOLD_AMPS
            && (now - lastShotTimestampSec) > SHOT_DEBOUNCE_SEC;

    if (shotCondition && !shotInProgress) {
      inputs.shotCount++;
      lastShotTimestampSec = now;
      shotInProgress = true;
    } else if (!shotCondition) {
      shotInProgress = false;
    }

    Logger.recordOutput("AnotherShooter/ShotCounter/DesiredRPM", targetRpm);
    Logger.recordOutput("AnotherShooter/ShotCounter/ActualRPM", actualRpm);
    Logger.recordOutput("AnotherShooter/ShotCounter/RPMDrop", rpmDrop);
    Logger.recordOutput("AnotherShooter/ShotCounter/AvgCurrentAmps", avgCurrentAmps);
    Logger.recordOutput("AnotherShooter/ShotCounter/Feeding", feeding);
    Logger.recordOutput("AnotherShooter/ShotCounter/ShotCondition", shotCondition);
    Logger.recordOutput("AnotherShooter/ShotCounter/ShotInProgress", shotInProgress);
    Logger.recordOutput("AnotherShooter/ShotCounter/Count", inputs.shotCount);
  }

  // Characterization methods
  public void setVoltage(double input) {
    io.setVoltage(input);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> setVoltage(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> setVoltage(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }
}
