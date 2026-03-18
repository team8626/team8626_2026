package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.AkitDrive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.frc2026.FieldConstants;

public class IndexAndShootCommand extends Command {
  private final Shooter shooter;
  private final Indexer indexer;
  private final AkitDrive drive;
  private final Hopper hopper;

  private final Translation3d target = FieldConstants.Hub.topCenterPoint;

  public IndexAndShootCommand(Shooter shooter, Hopper hopper, Indexer indexer, AkitDrive drive) {
    this.shooter = shooter;
    this.hopper = hopper;
    this.indexer = indexer;
    this.drive = drive;
    addRequirements(shooter, hopper, indexer);
  }

  @Override
  public void initialize() {
    shooter.start(ShooterCommandsUtil.calculateTreemapRPM(drive, target));
  }

  @Override
  public void execute() {
    AngularVelocity TreemapRPM = ShooterCommandsUtil.calculateTreemapRPM(drive, target);

    // If shooter is fast enough, run indexer to feed balls into shooter
    if (shooter.isAtGoal()) {
      indexer.start();
    }
    if (Constants.currentMode == Constants.Mode.SIM) {
      // In simulation, we can just pop fuel immediately when the indexer is running
      if (hopper.popFuel()) {
        RobotContainer.launchFuel(TreemapRPM);
      }
    }
    shooter.start(TreemapRPM);
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
