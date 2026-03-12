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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.Dimensions;
import frc.robot.Constants.Mode;
import frc.robot.commands.AlignToTargetCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IndexerStartCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.anotherShooter.AnotherShooter;
import frc.robot.subsystems.anotherShooter.AnotherShooterConstants;
import frc.robot.subsystems.anotherShooter.AnotherShooterIO;
import frc.robot.subsystems.anotherShooter.AnotherShooterIOSim;
import frc.robot.subsystems.anotherShooter.AnotherShooterIOSparkFlex;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants.Rebuilt_SwerveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOADIS16470;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSimTalonFX;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOSpark;
import frc.robot.subsystems.intakeLinkage.IntakeLinkage;
import frc.robot.subsystems.intakeLinkage.IntakeLinkageIO;
import frc.robot.subsystems.intakeLinkage.IntakeLinkageIOSim;
import frc.robot.subsystems.intakeRoller.IntakeRoller;
import frc.robot.subsystems.intakeRoller.IntakeRollerIO;
import frc.robot.subsystems.intakeRoller.IntakeRollerIOSim;
import frc.robot.subsystems.intakeRoller.IntakeRollerIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.util.FuelSim;
import java.util.Set;
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
  private final IntakeLinkage intakeLinkage;
  private final IntakeRoller intakeRoller;
  private final Hopper hopper;
  private final AnotherShooter anotherShooter;
  private final Climber climber;

  private final Vision vision;

  // Controller
  private static final CommandXboxController controller =
      new CommandXboxController(ControllerConstants.DRIVERPORT);
  // Commands
  private final TeleopDriveCommand teleopDrive;

  // Triggers for Bindings
  private static final Trigger indexTrigger = controller.a();
  private static final Trigger intakeRollerTrigger = controller.b();

  private static final Trigger shootTrigger = controller.leftTrigger();
  private static final Trigger indexAndShootTrigger = controller.rightTrigger();
  private static final Trigger intakeDeployTrigger = controller.leftBumper();

  private static final Trigger driverAimTrigger = controller.y();

  private static final Trigger climberReleaseTrigger = controller.povUp();
  private static final Trigger climberPullrigger = controller.povDown();

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Fuel Simulation
  public static FuelSim fuelSim;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    if (Constants.currentMode == Mode.SIM) {
      // Simulation: use physics sim IO (automatically detected via RobotBase.isReal())
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIOSimTalonFX(Rebuilt_SwerveConstants.FrontLeft.MODULE_CONSTANTS),
              new ModuleIOSimTalonFX(Rebuilt_SwerveConstants.FrontRight.MODULE_CONSTANTS),
              new ModuleIOSimTalonFX(Rebuilt_SwerveConstants.BackLeft.MODULE_CONSTANTS),
              new ModuleIOSimTalonFX(Rebuilt_SwerveConstants.BackRight.MODULE_CONSTANTS));
      index = new Indexer(new IndexerIOSim());
      intakeLinkage = new IntakeLinkage(new IntakeLinkageIOSim());
      intakeRoller = new IntakeRoller(new IntakeRollerIOSpark());
      hopper = new Hopper(new HopperIOSim());
      anotherShooter = new AnotherShooter(new AnotherShooterIOSim());
      climber = new Climber(new ClimberIOSim() {});

      vision =
          new Vision(
              new VisionIOSim(),
              (measurement) ->
                  drive.addVisionMeasurement(
                      measurement.pose, measurement.timestamp, measurement.stdDevs));
      vision.setPoseSupplier(drive::getPose);
      configureFuelSim();
      configureFuelSimRobot(hopper::ableToIntake, hopper::pushFuel);
    } else if (Constants.currentMode == Mode.REPLAY) {
      // Replay: no hardware IO
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});

      index = new Indexer(new IndexerIO() {});
      intakeLinkage = new IntakeLinkage(new IntakeLinkageIO() {});
      intakeRoller = new IntakeRoller(new IntakeRollerIO() {});
      hopper = new Hopper(new HopperIO() {});
      anotherShooter = new AnotherShooter(new AnotherShooterIO() {});
      climber = new Climber(new ClimberIO() {});

      vision = new Vision(new VisionIO() {}, (measurement) -> {});
    } else {
      switch (Constants.robot) {
        case TSUNAMI:
          // DEV bot on Spark
          drive =
              new Drive(
                  new GyroIOADIS16470(),
                  new ModuleIOSpark(0),
                  new ModuleIOSpark(1),
                  new ModuleIOSpark(2),
                  new ModuleIOSpark(3));

          index = new Indexer(new IndexerIOSim());
          intakeLinkage = new IntakeLinkage(new IntakeLinkageIOSim());
          intakeRoller = new IntakeRoller(new IntakeRollerIOSim());
          hopper = new Hopper(new HopperIOSim());
          anotherShooter = new AnotherShooter(new AnotherShooterIOSim());

          climber = new Climber(new ClimberIO() {});
          vision =
              new Vision(
                  new VisionIOPhotonVision(),
                  (measurement) ->
                      drive.addVisionMeasurement(
                          measurement.pose, measurement.timestamp, measurement.stdDevs));
          break;
        case REBUILT_COMPBOT:
          // Real robot, full hardware IO
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(Rebuilt_SwerveConstants.FrontLeft.MODULE_CONSTANTS),
                  new ModuleIOTalonFX(Rebuilt_SwerveConstants.FrontRight.MODULE_CONSTANTS),
                  new ModuleIOTalonFX(Rebuilt_SwerveConstants.BackLeft.MODULE_CONSTANTS),
                  new ModuleIOTalonFX(Rebuilt_SwerveConstants.BackRight.MODULE_CONSTANTS));

          index = new Indexer(new IndexerIOSpark() {});
          intakeLinkage = new IntakeLinkage(new IntakeLinkageIO() {});
          intakeRoller = new IntakeRoller(new IntakeRollerIOSpark());
          hopper = new Hopper(new HopperIO() {});
          climber = new Climber(new ClimberIO() {});

          anotherShooter = new AnotherShooter(new AnotherShooterIOSparkFlex());

          vision =
              new Vision(
                  new VisionIOPhotonVision(),
                  (measurement) ->
                      drive.addVisionMeasurement(
                          measurement.pose, measurement.timestamp, measurement.stdDevs));

          break;

        case REBUILT_DRIVE_ONLY:
          // Real robot, drive only
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(Rebuilt_SwerveConstants.FrontLeft.MODULE_CONSTANTS),
                  new ModuleIOTalonFX(Rebuilt_SwerveConstants.FrontRight.MODULE_CONSTANTS),
                  new ModuleIOTalonFX(Rebuilt_SwerveConstants.BackLeft.MODULE_CONSTANTS),
                  new ModuleIOTalonFX(Rebuilt_SwerveConstants.BackRight.MODULE_CONSTANTS));

          index = new Indexer(new IndexerIO() {});
          intakeLinkage = new IntakeLinkage(new IntakeLinkageIO() {});
          intakeRoller = new IntakeRoller(new IntakeRollerIO() {});
          hopper = new Hopper(new HopperIO() {});
          anotherShooter = new AnotherShooter(new AnotherShooterIOSparkFlex());

          climber = new Climber(new ClimberIO() {});

          vision = new Vision(new VisionIO() {}, (measurement) -> {});

          break;
        default:
          throw new IllegalStateException("Unexpected robot: " + Constants.robot);
      }
    }

    // Set up auto routines
    configurePPNamedCommands();
    configurePPEventTriggers();
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    configureSysIdRoutines();

    // Set up commands
    teleopDrive = new TeleopDriveCommand(drive, controller);

    // Configure the button bindings
    configureButtonBindings();

    // Configure named commands
    // configureNamedCommands();

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

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Align to camera's best AprilTag: POV Left = front left, POV Up = front right, POV Right =
    // rear right
    controller.povLeft().whileTrue(AlignToTargetCommand.alignToFrontLeftCamera(drive, vision));
    controller.povRight().whileTrue(AlignToTargetCommand.alignToFrontRightCamera(drive, vision));
    controller.povDown().whileTrue(AlignToTargetCommand.alignToRearRightCamera(drive, vision));

    // Run the intake roller at 500 RPM while the B button is held, stop when released
    // TODO: Should be replace with a proper command ("Start/Stop CollectCommand"), this is just for
    // testing
    intakeRollerTrigger
        .onTrue(Commands.runOnce(() -> intakeRoller.runVelocity(RPM.of(1000)), intakeRoller))
        .onFalse(Commands.runOnce(intakeRoller::stop, intakeRoller));

    // Toggle the intake linkage between deployed and stowed positions when the left bumper held
    // back to stowed position when released
    // TODO: Should be replace with a proper command ("Start/Stop CollectCommand"), this is just for
    // testing
    intakeDeployTrigger
        .whileTrue(Commands.runOnce(() -> intakeLinkage.setPosition(Degrees.of(90)), intakeLinkage))
        .onFalse(Commands.runOnce(() -> intakeLinkage.setPosition(Degrees.of(135)), intakeLinkage));

    // Run the climber motors
    // TODO: replace with proper commands once climber functionality is implemented, this is just
    // for testing
    climberReleaseTrigger.whileTrue(
        Commands.runOnce(() -> climber.runOpenLoop(Volts.of(6.0)), climber));
    climberPullrigger.whileTrue(
        Commands.runOnce(() -> climber.runOpenLoop(Volts.of(-6.0)), climber));

    // TODO: This is for testing only, replace with proper commands once anotherShooter and indexer
    // functionality are tested
    indexTrigger
        .onTrue(Commands.runOnce(() -> index.start(), index))
        .onFalse(Commands.runOnce(() -> index.stop(), index));

    // shootTrigger.whileTrue(new IndexAndShootCommand(anotherShooter, hopper, index, drive));
    // shootUpdateVelocityTrigger.onTrue(anotherShooter.updateVelocityCommand());
    shootTrigger
        .whileTrue(Commands.runOnce(() -> anotherShooter.start(RPM.of(2500)), anotherShooter))
        .onFalse(Commands.runOnce(() -> anotherShooter.stop(), anotherShooter));
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

    fuelSim.registerIntake(
        Dimensions.FULL_LENGTH.div(2).in(Meters),
        Dimensions.FULL_LENGTH.div(2).plus(Inches.of(7)).in(Meters),
        -Dimensions.FULL_WIDTH.div(2).in(Meters),
        Dimensions.FULL_WIDTH.div(2).in(Meters),
        ableToIntake,
        intakeCallback);
  }

  public static void launchFuel(AngularVelocity flywheelVelocity) {
    fuelSim.launchFuel(
        MetersPerSecond.of(
            flywheelVelocity.in(RadiansPerSecond)
                * AnotherShooterConstants.FLYWHEEL_RADIUS.in(Meters)),
        AnotherShooterConstants.ANOTHERSHOOTER_ANGLE,
        Degrees.of(AnotherShooterConstants.ANOTHERSHOOTER_OFFSET.getRotation().getAngle()),
        Meters.of(AnotherShooterConstants.ANOTHERSHOOTER_OFFSET.getZ()));
  }

  public static Trigger getHubAimTrigger() {
    return driverAimTrigger;
  }

  /**
   * Configure named commands to be identified by autos and paths. These are for instant commands
   * that can be triggered by auto builders and path planners.
   */
  private void configurePPNamedCommands() {

    NamedCommands.registerCommand(
        "AimAndDumpShort",
        Commands.defer(
                () -> new IndexerStartCommand(index), Set.of(index, /* anotherShooter, */ drive))
            .withTimeout(AutoConstants.DUMP_DURATION_SHORT.in(Seconds)));
    NamedCommands.registerCommand(
        "AimAndDumpMedium",
        Commands.defer(
                () -> new IndexerStartCommand(index), Set.of(index, /* anotherShooter, */ drive))
            .withTimeout(AutoConstants.DUMP_DURATION_MEDIUM.in(Seconds)));
    NamedCommands.registerCommand(
        "AimAndDumpLong",
        Commands.defer(
                () -> new IndexerStartCommand(index), Set.of(index, /* anotherShooter, */ drive))
            .withTimeout(AutoConstants.DUMP_DURATION_LONG.in(Seconds)));
  }

  /**
   * Configure event triggers to be identified by autos and paths. These are for non-instant
   * commands that need to be triggered by events in the middle of paths.
   */
  private void configurePPEventTriggers() {
    new EventTrigger("CollectStart")
        .whileTrue(Commands.print("--- PP Event Trigger - CollectStart"));
    new EventTrigger("CollectDone").whileTrue(Commands.print("--- PP Event Trigger - CollectDone"));
    new EventTrigger("RampUp").whileTrue(Commands.print("--- PP Event Trigger - RampUp"));
  }

  /**
   * Configure SysId routines to be identified by autos and paths. These will show up on the
   * dashboard and can be run to collect data for system identification.
   */
  private void configureSysIdRoutines() {
    SmartDashboard.putData(
        DriveCommands.wheelRadiusCharacterization(drive)
            .withName("Characterization/Drive Wheel Radius"));
    SmartDashboard.putData(
        DriveCommands.feedforwardCharacterization(drive)
            .withName("Characterization/Drive Simple FF"));
    SmartDashboard.putData(
        drive
            .sysIdQuasistatic(SysIdRoutine.Direction.kForward)
            .withName("Characterization/Drive Quasistatic Forward"));
    SmartDashboard.putData(
        drive
            .sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
            .withName("Characterization/Drive Quasistatic Reverse"));
    SmartDashboard.putData(
        drive
            .sysIdDynamic(SysIdRoutine.Direction.kForward)
            .withName("Characterization/Drive Dynamic Forward"));
    SmartDashboard.putData(
        drive
            .sysIdDynamic(SysIdRoutine.Direction.kReverse)
            .withName("Characterization/Drive Dynamic Reverse"));
    SmartDashboard.putData(
        anotherShooter
            .sysIdQuasistatic(SysIdRoutine.Direction.kForward)
            .withName("Characterization/AnotherShooter Quasistatic Forward"));
    SmartDashboard.putData(
        anotherShooter
            .sysIdDynamic(SysIdRoutine.Direction.kForward)
            .withName("Characterization/AnotherShooter Dynamic Forward"));
    SmartDashboard.putData(
        anotherShooter
            .sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
            .withName("Characterization/AnotherShooter Quasistatic Reverse"));
    SmartDashboard.putData(
        anotherShooter
            .sysIdDynamic(SysIdRoutine.Direction.kReverse)
            .withName("Characterization/AnotherShooter Dynamic Reverse"));
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
