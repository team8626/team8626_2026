package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intakeLinkage.IntakeLinkage;
import frc.robot.subsystems.intakeLinkage.IntakeLinkageConstants;
import frc.robot.subsystems.intakeRoller.IntakeRoller;
import frc.robot.subsystems.intakeRoller.IntakeRollerConstants;
import java.util.function.Supplier;

public class CollectCommand extends Command {

  private final IntakeLinkage linkage;
  private final IntakeRoller roller;

  private Supplier<AngularVelocity> targetVelocitySupplier;
  private Supplier<Angle> targetAngleSupplier;

  public CollectCommand(IntakeLinkage linkage, IntakeRoller roller) {
    this(() -> IntakeRollerConstants.DEFAULT_VELOCITY, linkage, roller);
  }

  public CollectCommand(
      Supplier<AngularVelocity> velocitySupplier, IntakeLinkage linkage, IntakeRoller roller) {
    this(() -> IntakeLinkageConstants.DEPLOY_ANGLE, velocitySupplier, linkage, roller);
  }

  public CollectCommand(
      Supplier<Angle> angleSupplier,
      Supplier<AngularVelocity> velocitySupplier,
      IntakeLinkage linkage,
      IntakeRoller roller) {
    this.linkage = linkage;
    this.roller = roller;
    this.targetVelocitySupplier = velocitySupplier;
    this.targetAngleSupplier = angleSupplier;
    addRequirements(linkage, roller);
  }

  @Override
  public void initialize() {
    linkage.setPosition(targetAngleSupplier.get());
    roller.start(targetVelocitySupplier.get());
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    linkage.stow();

    Commands.defer(
            () ->
                Commands.sequence(
                    Commands.waitSeconds(IntakeRollerConstants.STOP_DELAY.in(Seconds)),
                    Commands.runOnce(roller::stop, roller)),
            java.util.Set.of(roller))
        .schedule();
  }

  @Override
  public boolean isFinished() {
    return false; // Runs until interrupted
  }
}
