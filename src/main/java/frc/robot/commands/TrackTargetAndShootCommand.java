package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ShooterCommandsUtil.ShooterData;
import frc.robot.subsystems.anotherShooter.AnotherShooter;
import frc.robot.subsystems.anotherShooter.AnotherShooterConstants;
import frc.robot.subsystems.drive.AkitDrive;
import frc.robot.subsystems.indexer.Indexer;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.frc2026.FieldConstants;

public class TrackTargetAndShootCommand extends Command {
  private final Indexer indexer;
  private final AnotherShooter shooter;
  private final AkitDrive drive;
  private final Supplier<Translation3d> targetSupplier;

  public TrackTargetAndShootCommand(Indexer indexer, AnotherShooter shooter, AkitDrive drive) {
    this(() -> FieldConstants.Hub.topCenterPoint, indexer, shooter, drive);
  }

  public TrackTargetAndShootCommand(
      Supplier<Translation3d> targetSupplier,
      Indexer indexer,
      AnotherShooter shooter,
      AkitDrive drive) {
    this.indexer = indexer;
    this.shooter = shooter;
    this.drive = drive;
    this.targetSupplier = targetSupplier;
    addRequirements(indexer, shooter);
  }

  @Override
  public void initialize() {
    indexer.stop();
  }

  @Override
  public void execute() {
    ShooterData shot = ShooterCommandsUtil.calculateRPMToTarget(drive, targetSupplier.get());
    AngularVelocity requiredVelocity = shot.velocityShooter();
    AngularVelocity currentVelocity = shooter.getVelocity();

    shooter.start(requiredVelocity);

    boolean shooterAtSpeed =
        Math.abs(currentVelocity.in(RPM) - requiredVelocity.in(RPM))
            <= AnotherShooterConstants.VELOCITY_TOLERANCE.in(RPM);

    if (shooterAtSpeed) {
      indexer.start(shot.velocityIndexer());
    } else {
      indexer.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    indexer.stop();

    Commands.defer(
            () ->
                Commands.sequence(
                    Commands.waitSeconds(AnotherShooterConstants.STOP_DELAY.in(Seconds)),
                    Commands.runOnce(shooter::stop, shooter)),
            Set.of(shooter))
        .schedule();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
