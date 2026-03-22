package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.anotherShooter.AnotherShooter;
import frc.robot.subsystems.anotherShooter.AnotherShooterConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AnotherShooterRampupCommand extends Command {
  private final AnotherShooter shooter;
  private final Supplier<AngularVelocity> velocitySupplier;
  private static final AngularVelocity DEFAULT_VELOCITY = AnotherShooterConstants.DEFAULT_VELOCITY;

  private final Timer timer = new Timer();
  private boolean logged = false;

  public AnotherShooterRampupCommand(AnotherShooter new_shooter) {
    this(() -> DEFAULT_VELOCITY, new_shooter);
  }

  public AnotherShooterRampupCommand(
      Supplier<AngularVelocity> velocitySupplier, AnotherShooter shooter) {
    this.shooter = shooter;
    this.velocitySupplier = velocitySupplier;

    // DO NOT USE REQUITEMENTS, THIS BLOCKS AUTOs
    // addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.start(velocitySupplier.get());
    timer.restart();
    logged = false;
  }

  @Override
  public void execute() {
    Logger.recordOutput("AnotherShooter/Ramp/ElapsedSec", timer.get());
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    if (!logged) {
      Logger.recordOutput("AnotherShooter/Ramp/TimedOutSec", timer.get());
    }
    if (interrupted) {
      shooter.stop();
    }
  }

  @Override
  public boolean isFinished() {
    AngularVelocity currentRPM = shooter.getVelocity();

    boolean atSetpoint =
        Math.abs(currentRPM.in(RPM) - velocitySupplier.get().in(RPM))
            <= AnotherShooterConstants.VELOCITY_TOLERANCE.in(RPM);

    if (atSetpoint && !logged) {
      Logger.recordOutput("AnotherShooter/Ramp/TimeToSetpointSec", timer.get());
      logged = true;
    }

    return atSetpoint;
  }
}
