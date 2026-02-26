package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;

public class IndexerStopCommand extends Command {

  private final Indexer spindexer;

  public IndexerStopCommand(Indexer indexer) {
    this.spindexer = indexer;
    addRequirements(indexer);
  }

  @Override
  public void initialize() {
    spindexer.stop();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false; // Runs until interrupted
  }
}
