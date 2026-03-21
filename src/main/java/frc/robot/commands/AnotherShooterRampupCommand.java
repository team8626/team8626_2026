package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.anotherShooter.AnotherShooter;
import frc.robot.subsystems.anotherShooter.AnotherShooterConstants;
import java.util.function.Supplier;

public class AnotherShooterRampupCommand extends Command {
  private final AnotherShooter shooter;
  private final Supplier<AngularVelocity> velocitySupplier;
  private static final AngularVelocity DEFAULT_VELOCITY = AnotherShooterConstants.DEFAULT_VELOCITY;

  public AnotherShooterRampupCommand(AnotherShooter new_shooter) {
    this(() -> DEFAULT_VELOCITY, new_shooter);
  }

  public AnotherShooterRampupCommand(
      Supplier<AngularVelocity> velocitySupplier, AnotherShooter shooter) {
    this.shooter = shooter;
    this.velocitySupplier = velocitySupplier;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.start(velocitySupplier.get());
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      shooter.stop();
    }
  }

  @Override
  public boolean isFinished() {
    boolean atSetpoint = false;

    AngularVelocity currentRPM = shooter.getVelocity();

    atSetpoint =
        Math.abs(Math.abs(currentRPM.in(RPM)) - velocitySupplier.get().in(RPM))
            <= AnotherShooterConstants.VELOCITY_TOLERANCE.in(RPM);

    return atSetpoint;
  }
}
