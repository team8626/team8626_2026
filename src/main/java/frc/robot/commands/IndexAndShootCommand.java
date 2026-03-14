package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

public class IndexAndShootCommand extends Command {
  private final Shooter shooter;
  private final Indexer indexer;
  private final Drive drive;
  private final Hopper hopper;

  public IndexAndShootCommand(Shooter shooter, Hopper hopper, Indexer indexer, Drive drive) {
    this.shooter = shooter;
    this.hopper = hopper;
    this.indexer = indexer;
    this.drive = drive;
    addRequirements(shooter, hopper, indexer);
  }

  @Override
  public void initialize() {
    shooter.start(ShooterCommandsUtil.calculateTreemapRPM(ShooterCommandsUtil.getDistToHub(drive)));
  }

  @Override
  public void execute() {
    // If shooter is fast enough, run indexer to feed balls into shooter
    if (shooter.isAtGoal()) {
      indexer.start();
    }
    if (Constants.currentMode == Constants.Mode.SIM) {
      // In simulation, we can just pop fuel immediately when the indexer is running
      if (hopper.popFuel()) {
        RobotContainer.launchFuel(
            ShooterCommandsUtil.calculateTreemapRPM(ShooterCommandsUtil.getDistToHub(drive)));
      }
    }
    shooter.start(ShooterCommandsUtil.calculateTreemapRPM(ShooterCommandsUtil.getDistToHub(drive)));

    Logger.recordOutput(
        "getShooterVelocityToTarget/Robot Distance to Hub",
        (float) ShooterCommandsUtil.getDistToHub(drive),
        "feet");
    Logger.recordOutput(
        "getShooterVelocityToTarget/Calculated Shooter Velocity (based on treemap)",
        (float)
            ShooterCommandsUtil.calculateTreemapRPM(ShooterCommandsUtil.getDistToHub(drive))
                .in(RPM),
        "RPM");
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    indexer.stop();
  }

  @Override
  public boolean isFinished() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      // In simulation, we can end the command once we've popped all the fuel
      return hopper.isEmpty();
    }
    return false;
  }
}
