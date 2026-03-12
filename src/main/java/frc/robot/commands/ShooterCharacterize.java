package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.anotherShooter.*;

public class ShooterCharacterize extends Command {
  private final AnotherShooter shooter;
  private final Timer timer = new Timer();

  private static final double rampRate = 0.25; // volts per second
  private static final double maxVoltage = 8.0;

  public ShooterCharacterize(AnotherShooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    double voltage = rampRate * timer.get();

    if (voltage > maxVoltage) {
      voltage = maxVoltage;
    }

    shooter.setVoltage(voltage);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return timer.get() * rampRate >= maxVoltage;
  }
}
