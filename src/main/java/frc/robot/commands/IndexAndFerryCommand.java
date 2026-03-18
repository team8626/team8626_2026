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
import org.littletonrobotics.frc2026.util.geometry.AllianceFlipUtil;

public class IndexAndFerryCommand extends Command {
  private final Shooter shooter;
  private final Indexer indexer;
  private final AkitDrive drive;
  private final Hopper hopper;

  private Translation3d FerryTarget;

  // TODO: move these constants to field constances
  private final Translation3d depotFerryTarget =
      AllianceFlipUtil.apply(new Translation3d(2.5, 6, 0));
  private final Translation3d outpostFerryTarget =
      AllianceFlipUtil.apply(new Translation3d(2.5, 1.75, 0));

  public IndexAndFerryCommand(Shooter shooter, Hopper hopper, Indexer indexer, AkitDrive drive) {
    this.shooter = shooter;
    this.hopper = hopper;
    this.indexer = indexer;
    this.drive = drive;
    addRequirements(shooter, hopper, indexer);
  }

  @Override
  public void initialize() {
    if (AllianceFlipUtil.shouldFlip()) {
      FerryTarget =
          (drive.getPose().getY() > FieldConstants.fieldWidth / 2)
              ? outpostFerryTarget
              : depotFerryTarget;
    } else {
      FerryTarget =
          (drive.getPose().getY() > FieldConstants.fieldWidth / 2)
              ? depotFerryTarget
              : outpostFerryTarget;
    }
    shooter.start(ShooterCommandsUtil.calculateTreemapRPM(drive, FerryTarget));
  }

  @Override
  public void execute() {
    AngularVelocity TreemapRPM = ShooterCommandsUtil.calculateTreemapRPM(drive, FerryTarget);
    // If shooter is fast enough, run indexer to feed balls into shooter
    if (drive.getPose().getY() > 4) {
      FerryTarget = depotFerryTarget;
    } else {
      FerryTarget = outpostFerryTarget;
    }
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
    return false; // TODO: implement actual logic to determine when finished
  }
}
