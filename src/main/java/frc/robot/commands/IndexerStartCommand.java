package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import java.util.function.Supplier;

public class IndexerStartCommand extends Command {

  private final Indexer spindexer;
  private Supplier<AngularVelocity> targetVelocitySupplier;

  public IndexerStartCommand(Indexer indexer) {
    this(() -> IndexerConstants.DEFAULT_VELOCITY, indexer);
  }

  public IndexerStartCommand(Supplier<AngularVelocity> velocitySupplier, Indexer indexer) {
    this.spindexer = indexer;
    this.targetVelocitySupplier = velocitySupplier;
    addRequirements(indexer);
  }

  @Override
  public void initialize() {
    spindexer.runVelocity(targetVelocitySupplier.get());
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    spindexer.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // Runs until interrupted
  }

  /**
   * Update the target velocity supplier. This can be used to change the target velocity on the fly.
   *
   * @param velocitySupplier
   */
  public void setTargetVelocity(Supplier<AngularVelocity> velocitySupplier) {
    this.targetVelocitySupplier = velocitySupplier;
    this.initialize();
  }
}
