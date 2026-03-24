package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.anotherShooter.AnotherShooter;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intakeLinkage.IntakeLinkage;
import frc.robot.subsystems.intakeLinkage.IntakeLinkageConstants;
import frc.robot.subsystems.intakeRoller.IntakeRoller;
import frc.robot.subsystems.intakeRoller.IntakeRollerConstants;

/** Class to run various system checks on the robot. */
public class SystemChecks {
  private final IntakeRoller roller;
  private final IntakeLinkage linkage;
  private final Indexer indexer;
  private final AnotherShooter shooter;
  private final Climber climber;

  public SystemChecks(
      IntakeLinkage linkage,
      IntakeRoller roller,
      Indexer indexer,
      AnotherShooter shooter,
      Climber climber) {
    this.roller = roller;
    this.linkage = linkage;
    this.indexer = indexer;
    this.shooter = shooter;
    this.climber = climber;

    SmartDashboard.putData("System Checks/Intake Sequence", intake());
    SmartDashboard.putData("System Checks/Plow Sequence", plow());
    SmartDashboard.putData("System Checks/Shoot Sequence", shoot());

    SmartDashboard.putData("System Checks/Linkage", linkage());
    SmartDashboard.putData("System Checks/Rollers", rollers());
    SmartDashboard.putData("System Checks/Indexer", indexer());
    SmartDashboard.putData("System Checks/Shooter", shooter());
    SmartDashboard.putData("System Checks/Climber", climber());
  }

  public Command linkage() {
    return Commands.sequence(
            Commands.waitSeconds(1.0),
            Commands.runOnce(linkage::deploy, linkage),
            Commands.waitSeconds(2.0),
            Commands.runOnce(linkage::stow, linkage),
            Commands.waitSeconds(2.0),
            new AgitateCommand(linkage, roller).withTimeout(3.0),
            Commands.runOnce(linkage::stow, linkage))
        .finallyDo(linkage::stow)
        .withName("SystemCheck/Linkage");
  }

  public Command rollers() {
    return Commands.sequence(
            Commands.waitSeconds(1.0),
            Commands.startEnd(() -> roller.start(RPM.of(500)), roller::stop, roller)
                .withTimeout(3.0),
            Commands.waitSeconds(1.0),
            Commands.startEnd(() -> roller.start(RPM.of(-100)), roller::stop, roller)
                .withTimeout(3.0))
        .finallyDo(roller::stop)
        .withName("SystemCheck/Rollers");
  }

  /**
   * Run the indexer at 5 RPS for 3 seconds, then stop for 1 second, then run at default RPS again
   * for 3 seconds.
   */
  public Command indexer() {
    return Commands.sequence(
            Commands.waitSeconds(1.0),
            Commands.startEnd(() -> indexer.start(RotationsPerSecond.of(5)), indexer::stop, indexer)
                .withTimeout(3.0),
            Commands.waitSeconds(1.0),
            Commands.startEnd(indexer::start, indexer::stop, indexer).withTimeout(3.0))
        .finallyDo(indexer::stop)
        .withName("SystemCheck/Indexer");
  }

  /**
   * Ramp the shooter to 500 RPM, hold for 3 seconds, stop for 1 second, then ramp to default RPM
   * and hold for 3 seconds.
   */
  public Command shooter() {
    return Commands.sequence(
            Commands.waitSeconds(1.0),
            new AnotherShooterRampupCommand(() -> RPM.of(500), shooter).withTimeout(3.0),
            Commands.waitSeconds(3.0),
            Commands.runOnce(shooter::stop, shooter),
            Commands.waitSeconds(1.0),
            new AnotherShooterRampupCommand(shooter).withTimeout(3.0),
            Commands.waitSeconds(3.0))
        .finallyDo(shooter::stop)
        .withName("SystemCheck/Shooter");
  }

  /** Zero climber, then extend, then stow. */
  public Command climber() {
    return Commands.sequence(
            Commands.waitSeconds(1.0),
            climber.zero(),
            Commands.waitSeconds(1.0),
            climber.extend(),
            Commands.waitSeconds(1.0),
            climber.stow())
        .withName("SystemCheck/Climber");
  }

  /**
   * Run intake sequence: deploy linkage, run rollers at default RPM for 3 seconds, then stow
   * linkage.
   */
  public Command intake() {
    return Commands.sequence(
            Commands.waitSeconds(1.0),
            Commands.runOnce(linkage::deploy, linkage),
            Commands.startEnd(
                    () -> roller.start(IntakeRollerConstants.DEFAULT_VELOCITY),
                    roller::stop,
                    roller)
                .withTimeout(3.0),
            Commands.runOnce(linkage::stow, linkage))
        .finallyDo(
            () -> {
              roller.stop();
              linkage.stow();
            })
        .withName("SystemCheck/Intake");
  }

  /** Run plow sequence: set plow angle, run rollers at plow velocity for 3 seconds, then stow. */
  public Command plow() {
    return Commands.sequence(
            Commands.waitSeconds(1.0),
            Commands.runOnce(() -> linkage.setPosition(IntakeLinkageConstants.PLOW_ANGLE), linkage),
            Commands.startEnd(
                    () -> roller.start(IntakeRollerConstants.PLOW_VELOCITY), roller::stop, roller)
                .withTimeout(3.0),
            Commands.runOnce(linkage::stow, linkage))
        .finallyDo(
            () -> {
              roller.stop();
              linkage.stow();
            })
        .withName("SystemCheck/Plow");
  }

  /**
   * Ramp shooter to default RPM, then run indexer and agitator for 3 seconds, then stop everything.
   */
  public Command shoot() {
    return Commands.sequence(
            Commands.waitSeconds(1.0),
            new AnotherShooterRampupCommand(shooter).withTimeout(3.0),
            Commands.parallel(
                    Commands.startEnd(indexer::start, indexer::stop, indexer).withTimeout(3.0),
                    new AgitateCommand(linkage, roller).withTimeout(3.0))
                .withTimeout(3.0))
        .finallyDo(
            () -> {
              indexer.stop();
              shooter.stop();
              linkage.stow();
            })
        .withName("SystemCheck/Shoot");
  }
}
