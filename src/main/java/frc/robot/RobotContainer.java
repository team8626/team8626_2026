// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.Dimensions;
import frc.robot.commands.AlignToTargetCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IndexerCommands;
import frc.robot.commands.IndexerStartCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants.Rebuilt_SwerveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOADIS16470;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSimTalonFX;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.util.FuelSim;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public final Drive drive;
  private final Indexer index;
  private final Vision vision;

  // Controller
  private static final CommandXboxController controller =
      new CommandXboxController(ControllerConstants.DRIVERPORT);
  // Commands
  private final TeleopDriveCommand teleopDrive;

  // Bindings
  private final Trigger indexTrigger = controller.x();

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Fuel Simulation
  public FuelSim fuelSim;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.robot) {
      case TSUNAMI:
        // DEV bot on Spark, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOADIS16470(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        index = new Indexer(new IndexerIOSpark());
        vision =
            new Vision(
                new VisionIOPhotonVision(),
                (measurement) ->
                    drive.addVisionMeasurement(
                        measurement.pose, measurement.timestamp, measurement.stdDevs));
        break;
      case REBUILT_COMPBOT:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(Rebuilt_SwerveConstants.FrontLeft.MODULE_CONSTANTS),
                new ModuleIOTalonFX(Rebuilt_SwerveConstants.FrontRight.MODULE_CONSTANTS),
                new ModuleIOTalonFX(Rebuilt_SwerveConstants.BackLeft.MODULE_CONSTANTS),
                new ModuleIOTalonFX(Rebuilt_SwerveConstants.BackRight.MODULE_CONSTANTS));
        index = new Indexer(new IndexerIO() {});
        vision =
            new Vision(
                new VisionIOPhotonVision(),
                (measurement) ->
                    drive.addVisionMeasurement(
                        measurement.pose, measurement.timestamp, measurement.stdDevs));
        break;

      case SIMBOT:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                // new ModuleIOSimSpark() {},
                // new ModuleIOSimSpark() {},
                // new ModuleIOSimSpark() {},
                // new ModuleIOSimSpark() {});
                new ModuleIOSimTalonFX(Rebuilt_SwerveConstants.FrontLeft.MODULE_CONSTANTS),
                new ModuleIOSimTalonFX(Rebuilt_SwerveConstants.FrontRight.MODULE_CONSTANTS),
                new ModuleIOSimTalonFX(Rebuilt_SwerveConstants.BackLeft.MODULE_CONSTANTS),
                new ModuleIOSimTalonFX(Rebuilt_SwerveConstants.BackRight.MODULE_CONSTANTS));
        index = new Indexer(new IndexerIOSim());
        vision =
            new Vision(
                new VisionIOSim(),
                (measurement) ->
                    drive.addVisionMeasurement(
                        measurement.pose, measurement.timestamp, measurement.stdDevs));
        vision.setPoseSupplier(drive::getPose); // Provide current pose for simulation
        configureFuelSim();
        configureFuelSimRobot(() -> false, () -> {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        index = new Indexer(new IndexerIO() {});
        vision = new Vision(new VisionIO() {}, (measurement) -> {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Set up commands
    teleopDrive = new TeleopDriveCommand(drive, controller);

    // Configure the button bindings
    configureButtonBindings();

    // Configure named commands
    configureNamedCommands();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(teleopDrive);
    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // Run the indexer for 8 seconds at default velocity
    controller.y().onTrue(IndexerCommands.runForDuration(index, 8.0));

    // Align to front camera's best AprilTag (POV-Up) or back camera's best (POV-Down)
    controller.povUp().whileTrue(AlignToTargetCommand.alignToFrontCamera(drive, vision));
    controller.povDown().whileTrue(AlignToTargetCommand.alignToBackCamera(drive, vision));

    indexTrigger.toggleOnTrue(new IndexerStartCommand(this.index));
  }

  private void configureFuelSim() {
    fuelSim = new FuelSim();
    fuelSim.spawnStartingFuel();

    fuelSim.start();
    SmartDashboard.putData(
        Commands.runOnce(
                () -> {
                  fuelSim.clearFuel();
                  fuelSim.spawnStartingFuel();
                })
            .withName("Reset Fuel")
            .ignoringDisable(true));
  }

  private void configureFuelSimRobot(BooleanSupplier ableToIntake, Runnable intakeCallback) {
    fuelSim.registerRobot(
        Dimensions.FULL_WIDTH.in(Meters),
        Dimensions.FULL_LENGTH.in(Meters),
        Dimensions.BUMPER_HEIGHT.in(Meters),
        drive::getPose,
        drive::getFieldSpeeds);
    // fuelSim.registerIntake(
    //         -Dimensions.FULL_LENGTH.div(2).in(Meters),
    //         Dimensions.FULL_LENGTH.div(2).in(Meters),
    //         -Dimensions.FULL_WIDTH.div(2).plus(Inches.of(7)).in(Meters),
    //         -Dimensions.FULL_WIDTH.div(2).in(Meters),
    //         intakes.right.deployedTrigger.and(ableToIntake),
    //         intakeCallback);
    // fuelSim.registerIntake(
    //         -Dimensions.FULL_LENGTH.div(2).in(Meters),
    //         Dimensions.FULL_LENGTH.div(2).in(Meters),
    //         Dimensions.FULL_WIDTH.div(2).in(Meters),
    //         Dimensions.FULL_WIDTH.div(2).plus(Inches.of(7)).in(Meters),
    //         intakes.left.deployedTrigger.and(ableToIntake),
    //         intakeCallback);
  }

  public static Trigger getHubAimTrigger() { // TODO: might need to change the button for this
    return controller.x();
  }

  /** Configure named commands to be identified by autos and paths. */
  private void configureNamedCommands() {
    NamedCommands.registerCommand(
        "RunIndexerFor8Seconds", IndexerCommands.runForDuration(index, 8.0));
    NamedCommands.registerCommand(
        "AlignToFrontCamera", AlignToTargetCommand.alignToFrontCamera(drive, vision));
    NamedCommands.registerCommand(
        "AlignToBackCamera", AlignToTargetCommand.alignToBackCamera(drive, vision));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
