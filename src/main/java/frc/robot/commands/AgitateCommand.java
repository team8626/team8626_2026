package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeLinkage.IntakeLinkage;
import frc.robot.subsystems.intakeLinkage.IntakeLinkageConstants;
import frc.robot.subsystems.intakeRoller.IntakeRoller;
import frc.robot.subsystems.intakeRoller.IntakeRollerConstants;
import java.util.function.Supplier;

public class AgitateCommand extends Command {

  private final IntakeLinkage linkage;
  private final IntakeRoller rollers;

  private Supplier<Angle> targetAngleSupplier;

  private final Timer timer = new Timer();
  private boolean agitating = false;

  public AgitateCommand(
      Supplier<Angle> angleSupplier, IntakeLinkage linkage, IntakeRoller rollers) {
    this.linkage = linkage;
    this.rollers = rollers;
    this.targetAngleSupplier = angleSupplier;

    addRequirements(linkage);
  }

  public AgitateCommand(IntakeLinkage linkage, IntakeRoller rollers) {
    this(() -> IntakeLinkageConstants.DEPLOY_ANGLE, linkage, rollers);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    if (timer.hasElapsed(IntakeLinkageConstants.AGITATE_PERIOD)) {
      agitating = !agitating;
      linkage.setPosition(
          agitating
              ? IntakeLinkageConstants.AGITAGE_IN_ANGLE
              : IntakeLinkageConstants.AGITAGE_OUT_ANGLE);
      rollers.start(agitating ? IntakeRollerConstants.AGITATE_VELOCITY : RPM.of(0));

      timer.reset();
      timer.start();
    }
  }

  @Override
  public void end(boolean interrupted) {
    // linkage.stow();
    rollers.stop(); // TODO: See if this is needed
    linkage.setPosition(targetAngleSupplier.get());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
