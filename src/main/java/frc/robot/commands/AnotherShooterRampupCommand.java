package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.anotherShooter.AnotherShooter;
import frc.robot.subsystems.anotherShooter.AnotherShooterConstants;
import java.util.function.Supplier;

public class AnotherShooterRampupCommand extends Command {
  private final AnotherShooter shooter;
  private final Supplier<AngularVelocity> velocitySupplier;
  private static final AngularVelocity DEFAULT_VELOCITY = AnotherShooterConstants.DEFAULT_VELOCITY;

  public AnotherShooterRampupCommand(AnotherShooter new_shooter) {
    this(() -> DEFAULT_VELOCITY, new_shooter);
  }

  public AnotherShooterRampupCommand(
      Supplier<AngularVelocity> velocitySupplier, AnotherShooter shooter) {
    this.shooter = shooter;
    this.velocitySupplier = velocitySupplier;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.start(velocitySupplier.get());
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      shooter.stop();
    }
  }

  @Override
  public boolean isFinished() {
    boolean atSetpoint = false;

    AngularVelocity currentRPM = shooter.getVelocity();

    atSetpoint =
        Math.abs(Math.abs(currentRPM.in(RPM)) - velocitySupplier.get().in(RPM))
            <= AnotherShooterConstants.VELOCITY_TOLERANCE.in(RPM);

    return atSetpoint;
  }
}

// public class CoralShooterRampUp extends CS_Command {
//   private CoralShooterSubsystem mortar;
//   private boolean overrideRPM = false;
//   private AngularVelocity dashboardRPMLeft = RPM.of(0);
//   private AngularVelocity dashboardRPMRight = RPM.of(0);
//   private Supplier<AngularVelocity> desiredRpmLeft;
//   private Supplier<AngularVelocity> desiredRpmRight;
//   private final AngularVelocity RPMTolerance = CoralShooterConstants.RPMTolerance;
//   private String coralPresetName = "";

//   public CoralShooterRampUp() {
//     this(PresetManager.getCoralPreset());
//   }

//   public CoralShooterRampUp(Supplier<CoralPreset> newPreset) {
//     mortar = RobotContainer.mortar;

//     desiredRpmLeft = () -> newPreset.get().RPMLeft;
//     desiredRpmRight = () -> newPreset.get().RPMRight;
//     coralPresetName = "From Supplier - " + newPreset.get().name;

//     SmartDashboard.putBoolean("Commands/CoralShooterRampUp/leftAtSetPoint", false);
//     SmartDashboard.putBoolean("Commands/CoralShooterRampUp/rightAtSetPoint", false);
//     addRequirements(mortar);
//     this.setTAGString("CORALSHOOTER_RAMPUP3");
//   }

//   public CoralShooterRampUp(
//       Supplier<AngularVelocity> newRPMLeft, Supplier<AngularVelocity> newRPMRight) {
//     mortar = RobotContainer.mortar;

//     desiredRpmLeft = newRPMLeft;
//     desiredRpmRight = newRPMRight;
//     coralPresetName =
//         "From DoubleSuppliers"
//             + desiredRpmLeft.get().in(RPM)
//             + " / "
//             + desiredRpmRight.get().in(RPM);

//     addRequirements(mortar);
//     this.setTAGString("CORALSHOOTER_RAMPUP3");
//   }

//   @Override
//   public void initialize() {
//     Commodore.setCommodoreState(CommodoreState.CORAL_SHOOT_RAMPUP);
//     Dashboard.setCoralState(GamePieceState.RAMPING_UP);

//     overrideRPM = SmartDashboard.getBoolean("Commands/CoralShooterRampUp/OverrideRPM", false);
//     dashboardRPMLeft =
//         RPM.of(SmartDashboard.getNumber("Commands/CoralShooterRampUp/ForcedRMPLeft", 0));
//     dashboardRPMRight =
//         RPM.of(SmartDashboard.getNumber("Commands/CoralShooterRampUp/ForcedRMPRight", 0));

//     if (overrideRPM) {
//       desiredRpmLeft = () -> dashboardRPMLeft;
//       desiredRpmRight = () -> dashboardRPMRight;
//     }

//     printf(
//         "(%s) RPM Left: %f, RPM Right: %f",
//         coralPresetName, desiredRpmLeft.get().in(RPM), desiredRpmRight.get().in(RPM));
//     mortar.startRampUp(desiredRpmLeft.get(), desiredRpmRight.get());
//   }

//   @Override
//   public void execute() {}

//   @Override
//   public void end(boolean interrupted) {
//     Dashboard.setCoralState(GamePieceState.IDLE);
//     if (interrupted) {
//       mortar.stopAll();
//     }
//   }

// }
