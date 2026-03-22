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
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.Dimensions;
import frc.robot.Constants.Mode;
import frc.robot.commands.AgitateCommand;
import frc.robot.commands.AnotherShooterRampupCommand;
import frc.robot.commands.CollectCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SystemChecks;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.TrackTargetAndShootCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.anotherShooter.AnotherShooter;
import frc.robot.subsystems.anotherShooter.AnotherShooterConstants;
import frc.robot.subsystems.anotherShooter.AnotherShooterIO;
import frc.robot.subsystems.anotherShooter.AnotherShooterIOSim;
import frc.robot.subsystems.anotherShooter.AnotherShooterIOSparkFlex;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOSpark;
import frc.robot.subsystems.drive.AkitDrive;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.DriveConstants.DriveSpeed;
import frc.robot.subsystems.drive.DriveConstants.Rebuilt_SwerveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSimTalonFX;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOSpark;
import frc.robot.subsystems.intakeLinkage.IntakeLinkage;
import frc.robot.subsystems.intakeLinkage.IntakeLinkageConstants;
import frc.robot.subsystems.intakeLinkage.IntakeLinkageIO;
import frc.robot.subsystems.intakeLinkage.IntakeLinkageIOSim;
import frc.robot.subsystems.intakeLinkage.IntakeLinkageIOSpark;
import frc.robot.subsystems.intakeRoller.IntakeRoller;
import frc.robot.subsystems.intakeRoller.IntakeRollerConstants;
import frc.robot.subsystems.intakeRoller.IntakeRollerIO;
import frc.robot.subsystems.intakeRoller.IntakeRollerIOSim;
import frc.robot.subsystems.intakeRoller.IntakeRollerIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
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
  public final AkitDrive akitDrive;
  private final Indexer index;
  private final IntakeLinkage intakeLinkage;
  private final IntakeRoller intakeRoller;
  private final Hopper hopper;
  private final AnotherShooter anotherShooter;

  public final CommandSwerveDrivetrain drivetrain;

  private final Climber climber;

  private final Vision vision;

  private final SystemChecks systemChecks;

  // Controller
  private static final CommandXboxController controller =
      new CommandXboxController(ControllerConstants.DRIVERPORT);
  private static final CommandXboxController operator =
      new CommandXboxController(ControllerConstants.OPERATORPORT);
  // Commands
  private final TeleopDriveCommand teleopDrive;

  // Triggers for Bindings
  private static final Trigger testIndexTrigger = operator.leftTrigger();
  private static final Trigger testShootTrigger = operator.rightTrigger();
  private static final Trigger testIntakeRollerTrigger = operator.rightBumper();
  private static final Trigger testIntakeDeployTrigger = operator.leftBumper();
  private static final Trigger testAgitateTrigger = operator.y();

  private static final Trigger collectTrigger = controller.leftBumper();
  private static final Trigger plowTrigger = controller.leftTrigger();
  private static final Trigger blurpTrigger = controller.y();
  private static final Trigger unjamTrigger = controller.x();

  private static final Trigger fixedRPMShootTrigger = controller.rightBumper();
  private static final Trigger aimAndShootTrigger = controller.rightTrigger();
  private static final Trigger collectAndShootTrigger = collectTrigger.and(aimAndShootTrigger);

  private static final Trigger hubAimTrigger = controller.b();
  private static final Trigger hubAimInPlaceTrigger = controller.a();

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Fuel Simulation
  public static FuelSim fuelSim;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    if (Constants.currentMode == Mode.SIM) {
      // Simulation: use physics sim IO (automatically detected via RobotBase.isReal())
      drivetrain = null; // Not Used here (this is for using Phoenix Tuner template)
      akitDrive =
          new AkitDrive(
              new GyroIO() {},
              new ModuleIOSimTalonFX(Rebuilt_SwerveConstants.FrontLeft.MODULE_CONSTANTS),
              new ModuleIOSimTalonFX(Rebuilt_SwerveConstants.FrontRight.MODULE_CONSTANTS),
              new ModuleIOSimTalonFX(Rebuilt_SwerveConstants.BackLeft.MODULE_CONSTANTS),
              new ModuleIOSimTalonFX(Rebuilt_SwerveConstants.BackRight.MODULE_CONSTANTS));
      index = new Indexer(new IndexerIOSim());
      intakeLinkage = new IntakeLinkage(new IntakeLinkageIOSim());
      intakeRoller = new IntakeRoller(new IntakeRollerIOSim());
      hopper = new Hopper(new HopperIOSim());
      anotherShooter = new AnotherShooter(new AnotherShooterIOSim());
      climber = new Climber(new ClimberIOSim() {});
      vision =
          new Vision(
              akitDrive::addVisionMeasurement,
              new VisionIOPhotonVisionSim(
                  VisionConstants.CAMERA_NAMES[0],
                  VisionConstants.CAMERA_TRANSFORMS[0],
                  akitDrive::getPose),
              new VisionIOPhotonVisionSim(
                  VisionConstants.CAMERA_NAMES[1],
                  VisionConstants.CAMERA_TRANSFORMS[1],
                  akitDrive::getPose),
              new VisionIOPhotonVisionSim(
                  VisionConstants.CAMERA_NAMES[2],
                  VisionConstants.CAMERA_TRANSFORMS[2],
                  akitDrive::getPose));
      //   vision.setPoseSupplier(drive::getPose);
      configureFuelSim();
      configureFuelSimRobot(hopper::ableToIntake, hopper::pushFuel);
    } else if (Constants.currentMode == Mode.REPLAY) {
      // Replay: no hardware IO
      drivetrain = null; // Not Used here (this is for using Phoenix Tuner template)
      akitDrive =
          new AkitDrive(
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

      vision =
          new Vision(
              akitDrive::addVisionMeasurement,
              new VisionIO() {},
              new VisionIO() {},
              new VisionIO() {});
    } else {
      switch (Constants.robot) {
        case REBUILT_PHOENIX:
          drivetrain = TunerConstants.createDrivetrain();
          akitDrive =
              new AkitDrive(
                  new GyroIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {});

          index = new Indexer(new IndexerIOSpark() {});
          intakeLinkage = new IntakeLinkage(new IntakeLinkageIOSpark() {});
          intakeRoller = new IntakeRoller(new IntakeRollerIOSpark() {});
          hopper = new Hopper(new HopperIO() {});
          anotherShooter = new AnotherShooter(new AnotherShooterIOSparkFlex());
          climber = new Climber(new ClimberIOSpark() {});

          vision =
              new Vision(
                  akitDrive::addVisionMeasurement,
                  new VisionIOPhotonVision(
                      VisionConstants.CAMERA_NAMES[0], VisionConstants.CAMERA_TRANSFORMS[0]),
                  new VisionIOPhotonVision(
                      VisionConstants.CAMERA_NAMES[1], VisionConstants.CAMERA_TRANSFORMS[1]),
                  new VisionIOPhotonVision(
                      VisionConstants.CAMERA_NAMES[2], VisionConstants.CAMERA_TRANSFORMS[2]));

          break;
        case REBUILT_AKIT:
          // Real robot, full hardware IO
          drivetrain = null; // Not Used here (this is for using Phoenix Tuner template)
          akitDrive =
              new AkitDrive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(Rebuilt_SwerveConstants.FrontLeft.MODULE_CONSTANTS),
                  new ModuleIOTalonFX(Rebuilt_SwerveConstants.FrontRight.MODULE_CONSTANTS),
                  new ModuleIOTalonFX(Rebuilt_SwerveConstants.BackLeft.MODULE_CONSTANTS),
                  new ModuleIOTalonFX(Rebuilt_SwerveConstants.BackRight.MODULE_CONSTANTS));

          index = new Indexer(new IndexerIOSpark() {});
          intakeLinkage = new IntakeLinkage(new IntakeLinkageIOSpark() {});
          intakeRoller = new IntakeRoller(new IntakeRollerIOSpark() {});
          hopper = new Hopper(new HopperIO() {});
          anotherShooter = new AnotherShooter(new AnotherShooterIOSparkFlex());
          climber = new Climber(new ClimberIOSpark() {});

          vision =
              new Vision(
                  akitDrive::addVisionMeasurement,
                  new VisionIOPhotonVision(
                      VisionConstants.CAMERA_NAMES[0], VisionConstants.CAMERA_TRANSFORMS[0]),
                  new VisionIOPhotonVision(
                      VisionConstants.CAMERA_NAMES[1], VisionConstants.CAMERA_TRANSFORMS[1]),
                  new VisionIOPhotonVision(
                      VisionConstants.CAMERA_NAMES[2], VisionConstants.CAMERA_TRANSFORMS[2]));

          break;

        default:
          throw new IllegalStateException("Unexpected robot: " + Constants.robot);
      }
    }

    // Setup System Checks
    systemChecks = new SystemChecks(intakeLinkage, intakeRoller, index, anotherShooter, climber);

    // Set up auto routines
    configurePPEventTriggers();
    configurePPNamedCommands();
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    configureSysIdRoutines();

    // Set up commands
    teleopDrive = new TeleopDriveCommand(akitDrive, controller);

    // Configure the button bindings
    configureDefaultCommands();
    configureButtonBindings();

    // Configure named commands
    // configureNamedCommands();

  }

  public void configureDefaultCommands() {

    switch (Constants.robot) {
      case REBUILT_PHOENIX:
        double MaxSpeed =
            1.0
                * TunerConstants.kSpeedAt12Volts.in(
                    MetersPerSecond); // kSpeedAt12Volts desired top speed
        double MaxAngularRate =
            RotationsPerSecond.of(0.75)
                .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

        SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric()
                .withDeadband(MaxSpeed * 0.1)
                .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                .withDriveRequestType(
                    DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(
                () ->
                    drive
                        .withVelocityX(
                            -controller.getLeftY()
                                * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(
                            -controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(
                            -controller.getRightX()
                                * MaxAngularRate) // Drive counterclockwise with negative X (left)
                ));
        break;

      case REBUILT_AKIT:
        akitDrive.setDefaultCommand(teleopDrive);
        break;

      default:
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // -------------------------------------------------------------- Collect
    //
    // Run the intake roller and moves the intake to collect position.
    // Also runs the drivetrain at a slow speed to help with intaking.
    //
    collectTrigger
        .and(aimAndShootTrigger.negate())
        .whileTrue(
            new CollectCommand(intakeLinkage, intakeRoller)
                .alongWith(teleopDrive.withSpeed(DriveSpeed.INTAKE))
                .withName("Collect Command"));

    // -------------------------------------------------------------- Plow
    //
    // Run the intake roller backwards and moves the intake to plow position
    //
    plowTrigger.whileTrue(
        new CollectCommand(
                () -> IntakeLinkageConstants.PLOW_ANGLE,
                () -> IntakeRollerConstants.PLOW_VELOCITY,
                intakeLinkage,
                intakeRoller)
            .withName("Plow Command"));

    // -------------------------------------------------------------- Just Shoot
    //
    // Run the shooter and indexer to dump fuel from the hopper.
    // The shooter will run at a fixed velocity.
    // Activates agitation
    //
    fixedRPMShootTrigger.whileTrue(
        Commands.sequence(new AnotherShooterRampupCommand(anotherShooter), feedShooterCommand())
            .withName("Just Shoot Command")
            .finallyDo(() -> stopShooting(AnotherShooterConstants.STOP_DELAY)));

    // -------------------------------------------------------------- Aim and Shoot
    //
    // Run the shooter and indexer to dump fuel from the hopper.
    // The robot will align to target.
    // The shooter will run at a velocity based on the distance to the target.
    // Activates agitation
    //
    aimAndShootTrigger
        .and(collectTrigger.negate())
        .whileTrue(
            teleopDrive.withHubLockThenX(
                Commands.parallel(
                        new TrackTargetAndShootCommand(index, anotherShooter, akitDrive),
                        new AgitateCommand(intakeLinkage))
                    .withName("Aim And Shoot Command")));

    // -------------------------------------------------------------- Collect and Shoot
    //
    // Run the intake, shooter, and indexer to collect fuel and dump it into the hub.
    // The robot will align to target while doing this.
    //
    collectAndShootTrigger.whileTrue(
        teleopDrive.withHubLock(
            teleopDrive.withSpeed(
                DriveSpeed.INTAKE,
                Commands.parallel(
                        new CollectCommand(intakeLinkage, intakeRoller),
                        new TrackTargetAndShootCommand(index, anotherShooter, akitDrive))
                    .withName("Collect And Shoot Command"))));

    // -------------------------------------------------------------- Unjam
    //
    // Run the shooter and indexer in reverse to unjam fuel.
    //
    unjamTrigger.whileTrue(
        Commands.parallel(
                Commands.runOnce(
                        () -> anotherShooter.start(AnotherShooterConstants.UNJAM_VELOCITY),
                        anotherShooter)
                    .andThen(Commands.idle(anotherShooter)),
                Commands.runOnce(() -> index.start(IndexerConstants.UNJAM_VELOCITY), index)
                    .andThen(Commands.idle(index)))
            .finallyDo(
                () -> {
                  anotherShooter.stop();
                  index.stop();
                })
            .withName("Unjamming Command"));

    // -------------------------------------------------------------- Blurp
    //
    // Open the intake and reverse the rollers.
    blurpTrigger.whileTrue(
        Commands.parallel(
                Commands.runOnce(
                        () -> intakeRoller.start(IntakeRollerConstants.BLURP_VELOCITY),
                        intakeRoller)
                    .andThen(Commands.idle(intakeRoller)),
                Commands.runOnce(
                        () -> intakeLinkage.setPosition(IntakeLinkageConstants.BLURP_ANGLE),
                        intakeLinkage)
                    .andThen(Commands.idle(intakeLinkage)))
            .finallyDo(
                () -> {
                  intakeRoller.stop();
                  intakeLinkage.setPosition(IntakeLinkageConstants.STOW_ANGLE);
                })
            .withName("Blurp Command"));

    // Align to camera's best AprilTag: POV Left = front left, POV Up = front right, POV Right =
    // rear right
    // controller.povLeft().whileTrue(AlignToTargetCommand.alignToFrontLeftCamera(drive, vision));
    // controller.povRight().whileTrue(AlignToTargetCommand.alignToFrontRightCamera(drive, vision));
    // controller.povDown().whileTrue(AlignToTargetCommand.alignToRearRightCamera(drive, vision));

    // --------------------------------------------------------------
    // Test commands for subsystems.
    // These are not intended for comp, just for testing on the practice bot and in sim.

    // Run the intake roller at the dashboard RPM
    testIntakeRollerTrigger.toggleOnTrue(
        Commands.startEnd(() -> intakeRoller.start(), () -> intakeRoller.stop(), intakeRoller)
            .withName("Start intakeRoller (Dashboard RPM)"));

    // Deploy/Stow the intake
    testIntakeDeployTrigger.toggleOnTrue(
        Commands.startEnd(
                () -> intakeLinkage.setPosition(IntakeLinkageConstants.DEPLOY_ANGLE),
                () -> intakeLinkage.setPosition(IntakeLinkageConstants.STOW_ANGLE),
                intakeLinkage)
            .withName("Deploy intakeLinkage (Dashboard RPM)"));

    // Fuel agitation
    testAgitateTrigger.whileTrue(new AgitateCommand(intakeLinkage));

    // Run the indexer at the dashboard RPM
    testIndexTrigger.toggleOnTrue(
        Commands.startEnd(() -> index.start(), () -> index.stop(), index)
            .withName("Start Indexer (Dashboard RPM)"));

    // Run the shooter at the dashboard RPM
    testShootTrigger.toggleOnTrue(
        Commands.startEnd(() -> anotherShooter.start(), () -> anotherShooter.stop(), anotherShooter)
            .withName("Start Shooter (Dashboard RPM)"));
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
        akitDrive::getPose,
        akitDrive::getFieldSpeeds);

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
    return hubAimTrigger;
  }

  public static Trigger getHubAimInPlaceTrigger() {
    return hubAimInPlaceTrigger;
  }

  /**
   * Configure named commands to be identified by autos and paths. These are for instant commands
   * that can be triggered by auto builders and path planners.
   */
  private void configurePPNamedCommands() {
    NamedCommands.registerCommand(
        "AimAndDumpShort",
        Commands.sequence(
                new AnotherShooterRampupCommand(anotherShooter).withTimeout(0.5),
                feedShooterCommand().withTimeout(AutoConstants.DUMP_DURATION_SHORT.in(Seconds)))
            .finallyDo(() -> stopShooting(AnotherShooterConstants.STOP_DELAY))
            .withName("AimAndDumpShort"));

    NamedCommands.registerCommand(
        "AimAndDumpMedium",
        Commands.sequence(
                new AnotherShooterRampupCommand(anotherShooter),
                Commands.deadline(
                    Commands.waitSeconds(AutoConstants.DUMP_DURATION_MEDIUM.in(Seconds)),
                    feedShooterCommand()))
            .finallyDo(() -> stopShooting(AnotherShooterConstants.STOP_DELAY))
            .withName("AimAndDumpMedium"));

    NamedCommands.registerCommand(
        "AimAndDumpLong",
        Commands.sequence(
                new AnotherShooterRampupCommand(anotherShooter),
                Commands.deadline(
                    Commands.waitSeconds(AutoConstants.DUMP_DURATION_MEDIUM.in(Seconds)),
                    feedShooterCommand()))
            .finallyDo(() -> stopShooting(AnotherShooterConstants.STOP_DELAY))
            .withName("AimAndDumpLong"));
  }

  /**
   * Configure event triggers to be identified by autos and paths. These are for non-instant
   * commands that need to be triggered by events in the middle of paths.
   */
  private void configurePPEventTriggers() {
    // Start collecting when the "CollectStart" event is triggered
    new EventTrigger("CollectStart")
        .onTrue(
            Commands.runOnce(
                    () -> {
                      intakeLinkage.setPosition(IntakeLinkageConstants.DEPLOY_ANGLE);
                      intakeRoller.start(IntakeRollerConstants.DEFAULT_VELOCITY);
                    })
                .withName("PP Collect Start"));

    // Stop collecting when the "CollectDone" event is triggered.
    new EventTrigger("CollectDone")
        .onTrue(
            Commands.runOnce(
                    () -> {
                      intakeLinkage.setPosition(IntakeLinkageConstants.STOW_ANGLE);
                      intakeRoller.stop();
                    })
                .withName("PP Collect Done"));

    // Ramp up the shooter when the "RampUp" event is triggered
    new EventTrigger("RampUp")
        .onTrue(
            Commands.parallel(
                Commands.print("=== RampUp FIRED ==="),
                new AnotherShooterRampupCommand(anotherShooter).withName("PP Ramp Up")));
  }

  /**
   * Configure SysId routines to be identified by autos and paths. These will show up on the
   * dashboard and can be run to collect data for system identification.
   */
  private void configureSysIdRoutines() {
    SmartDashboard.putData(
        DriveCommands.wheelRadiusCharacterization(akitDrive)
            .withName("Characterization/Drive Wheel Radius"));
    SmartDashboard.putData(
        DriveCommands.feedforwardCharacterization(akitDrive)
            .withName("Characterization/Drive Simple FF"));

    SmartDashboard.putData(
        akitDrive
            .sysIdQuasistatic(SysIdRoutine.Direction.kForward)
            .withName("Characterization/Drive Quasistatic Forward"));
    SmartDashboard.putData(
        akitDrive
            .sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
            .withName("Characterization/Drive Quasistatic Reverse"));
    SmartDashboard.putData(
        akitDrive
            .sysIdDynamic(SysIdRoutine.Direction.kForward)
            .withName("Characterization/Drive Dynamic Forward"));
    SmartDashboard.putData(
        akitDrive
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

    SmartDashboard.putData(
        index
            .sysIdQuasistatic(SysIdRoutine.Direction.kForward)
            .withName("Characterization/Indexer Quasistatic Forward"));
    SmartDashboard.putData(
        index
            .sysIdDynamic(SysIdRoutine.Direction.kForward)
            .withName("Characterization/Indexer Dynamic Forward"));
    SmartDashboard.putData(
        index
            .sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
            .withName("Characterization/Indexer Quasistatic Reverse"));
    SmartDashboard.putData(
        index
            .sysIdDynamic(SysIdRoutine.Direction.kReverse)
            .withName("Characterization/Indexer Dynamic Reverse"));

    SmartDashboard.putData(
        intakeRoller
            .sysIdQuasistatic(SysIdRoutine.Direction.kForward)
            .withName("Characterization/Rollers Quasistatic Forward"));
    SmartDashboard.putData(
        intakeRoller
            .sysIdDynamic(SysIdRoutine.Direction.kForward)
            .withName("Characterization/Rollers Dynamic Forward"));
    SmartDashboard.putData(
        intakeRoller
            .sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
            .withName("Characterization/Rollers Quasistatic Reverse"));
    SmartDashboard.putData(
        intakeRoller
            .sysIdDynamic(SysIdRoutine.Direction.kReverse)
            .withName("Characterization/Rollers Dynamic Reverse"));

    SmartDashboard.putData(
        intakeLinkage
            .sysIdQuasistatic(SysIdRoutine.Direction.kForward)
            .withName("Characterization/Linkage Quasistatic Forward"));
    SmartDashboard.putData(
        intakeLinkage
            .sysIdDynamic(SysIdRoutine.Direction.kForward)
            .withName("Characterization/Linkage Dynamic Forward"));
    SmartDashboard.putData(
        intakeLinkage
            .sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
            .withName("Characterization/Linkage Quasistatic Reverse"));
    SmartDashboard.putData(
        intakeLinkage
            .sysIdDynamic(SysIdRoutine.Direction.kReverse)
            .withName("Characterization/Linkage Dynamic Reverse"));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private Command feedShooterCommand() {
    return Commands.parallel(
        Commands.startEnd(
            () -> {
              anotherShooter.startFeeding();
              index.start();
            },
            () -> {
              anotherShooter.stopFeeding();
              index.stop();
            },
            index),
        new AgitateCommand(intakeLinkage));
  }

  private void stopShooting() {
    stopShooting(Seconds.of(0));
  }

  private void stopShooting(Time delay) {
    index.stop();
    anotherShooter.stopFeeding();
    intakeLinkage.stow();

    CommandScheduler.getInstance()
        .schedule(
            Commands.sequence(
                Commands.waitSeconds(delay.in(Seconds)),
                Commands.runOnce(anotherShooter::stop, anotherShooter)));
  }
}
