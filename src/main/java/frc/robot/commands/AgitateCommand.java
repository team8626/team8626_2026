package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeLinkage.IntakeLinkage;
import frc.robot.subsystems.intakeLinkage.IntakeLinkageConstants;
import frc.robot.subsystems.intakeRoller.IntakeRoller;
import frc.robot.subsystems.intakeRoller.IntakeRollerConstants;

public class AgitateCommand extends Command {

  private final IntakeLinkage linkage;
  private final IntakeRoller rollers;

  private final Timer timer = new Timer();
  private boolean agitating = false;

  public AgitateCommand(IntakeLinkage linkage, IntakeRoller rollers) {
    this.linkage = linkage;
    this.rollers = rollers;

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
      rollers.start(agitating ? IntakeRollerConstants.AGITATE_VELOCITY : RPM.of(0));

      timer.reset();
      timer.start();
    }
  }

  @Override
  public void end(boolean interrupted) {
    linkage.stow();
    rollers.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
