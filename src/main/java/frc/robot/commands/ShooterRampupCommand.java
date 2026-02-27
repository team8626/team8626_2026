package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.Supplier;

public class ShooterRampupCommand extends Command {
  private final Shooter shooter;
  private final Supplier<AngularVelocity> velocitySupplier;
  private static final AngularVelocity DEFAULT_VELOCITY =
      RPM.of(2000); // TODO: move to constants and find actual value

  public ShooterRampupCommand(Shooter new_shooter) {
    this(() -> DEFAULT_VELOCITY, new_shooter);
  }

  public ShooterRampupCommand(Supplier<AngularVelocity> velocitySupplier, Shooter shooter) {
    this.shooter = shooter;
    this.velocitySupplier = velocitySupplier;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.runVelocity(velocitySupplier.get());
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
    return shooter.isAtGoal();
  }
}
