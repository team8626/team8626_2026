package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeLinkage.IntakeLinkage;
import frc.robot.subsystems.intakeLinkage.IntakeLinkageConstants;

public class AgitateCommand extends Command {

  private final IntakeLinkage linkage;

  private final Timer timer = new Timer();
  private boolean agitating = false;

  public AgitateCommand(IntakeLinkage linkage) {
    this.linkage = linkage;
    addRequirements(linkage);
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
          agitating ? IntakeLinkageConstants.AGITAGE_ANGLE : IntakeLinkageConstants.STOW_ANGLE);
      timer.reset();
      timer.start();
    }
  }

  @Override
  public void end(boolean interrupted) {
    linkage.stow();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
