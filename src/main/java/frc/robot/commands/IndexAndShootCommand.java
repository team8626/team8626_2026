/** THIS COMMAND IS NOT COMPLETED YET */
/** WILL BE UPDATED IN FUTURE COMMITS */

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;

public class IndexAndShootCommand extends Command {
  private final Shooter shooter;
  private final Indexer indexer;
  private final Drive drive;

  private final double flywheelRadiousInInches = 2.0; // TODO: find actual value
  private final Angle shooterAngle = Degrees.of(38.0);
  private final Distance shooterXPosition = Inches.of(0.0);
  private final Distance shooterYPosition = Inches.of(0.0);
  private final Distance shooterZPosition = Inches.of(15);

  public IndexAndShootCommand(Shooter shooter, Indexer indexer, Drive drive) {
    this.shooter = shooter;
    this.indexer = indexer;
    this.drive = drive;
    addRequirements(shooter, indexer, drive);
  }

  @Override
  public void initialize() {
    new SequentialCommandGroup(
        new ShooterRampupCommand(shooter),
        IndexerCommands.runForDuration(
            indexer,
            1.2) // TODO: Link, make an IndexerCommand.run that starts the indexer and keeps it on
        // until a stop command is called
        );
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false; // TODO: implement actual logic to determine when finished
  }
}
