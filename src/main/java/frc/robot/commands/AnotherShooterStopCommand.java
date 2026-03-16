package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.anotherShooter.AnotherShooter;

public class AnotherShooterStopCommand extends Command {
  private final AnotherShooter shooter;

  public AnotherShooterStopCommand(AnotherShooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.stop();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true; // Command finishes immediately after stopping the shooter
  }
}
