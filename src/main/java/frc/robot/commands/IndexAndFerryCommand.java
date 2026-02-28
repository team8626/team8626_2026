package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;

public class IndexAndFerryCommand extends Command {
  private final Shooter shooter;
  private final Indexer indexer;
  private final Drive drive;
  private final Hopper hopper;

  private Translation3d FerryTarget;

  // TODO: move these constants to field constances
  private final Translation3d depotFerryTarget = new Translation3d(2.5, 6, 0);
  private final Translation3d outpostFerryTarget = new Translation3d(2.5, 1.75, 0);

  public IndexAndFerryCommand(Shooter shooter, Hopper hopper, Indexer indexer, Drive drive) {
    this.shooter = shooter;
    this.hopper = hopper;
    this.indexer = indexer;
    this.drive = drive;
    addRequirements(shooter, hopper, indexer);
  }

  @Override
  public void initialize() {
    if (drive.getPose().getY() > 4) {
      FerryTarget = depotFerryTarget;
    } else {
      FerryTarget = outpostFerryTarget;
    }
    shooter.runVelocity(
        ShooterCommandsUtil.getShooterVelocityToTarget(drive.getPose(), FerryTarget));
  }

  @Override
  public void execute() {
    // If shooter is fast enough, run indexer to feed balls into shooter
    if (drive.getPose().getY() > 4) {
      FerryTarget = depotFerryTarget;
    } else {
      FerryTarget = outpostFerryTarget;
    }
    if (shooter.isAtGoal()) {
      indexer.runVelocity(RPM.of(300)); // TODO: find actual value for indexer velocity
    }
    if (Constants.robot == RobotType.SIMBOT) {
      // In simulation, we can just pop fuel immediately when the indexer is running
      if (hopper.popFuel()) {
        RobotContainer.launchFuel(
            ShooterCommandsUtil.getShooterVelocityToTarget(drive.getPose(), FerryTarget));
      }
    }
    shooter.runVelocity(
        ShooterCommandsUtil.getShooterVelocityToTarget(drive.getPose(), FerryTarget));
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    indexer.stop();
  }

  @Override
  public boolean isFinished() {
    if (Constants.robot == RobotType.SIMBOT) {
      // In simulation, we can end the command once we've popped all the fuel
      return hopper.isEmpty();
    }
    return false; // TODO: implement actual logic to determine when finished
  }
}
