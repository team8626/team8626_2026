package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.MockCommandXboxController;
import frc.robot.subsystems.drive.MockGyroIO;
import frc.robot.subsystems.drive.MockModuleIO;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

/** Unit tests for the Drive subsystem. */
@Tag("unit")
public class TelelopDriveTest {
  private static final double DELTA = 1e-3;

  private MockGyroIO gyroIO;
  private MockModuleIO flModuleIO;
  private MockModuleIO frModuleIO;
  private MockModuleIO blModuleIO;
  private MockModuleIO brModuleIO;
  private Drive drive;
  private CommandXboxController controller;

  @BeforeAll
  static void initializeHAL() {
    // Initialize HAL once for all tests
    HAL.initialize(500, 0);
  }

  @BeforeEach
  void setUp() {
    gyroIO = new MockGyroIO();
    flModuleIO = new MockModuleIO();
    frModuleIO = new MockModuleIO();
    blModuleIO = new MockModuleIO();
    brModuleIO = new MockModuleIO();

    // Set default odometry data so periodic() doesn't fail
    setDefaultOdometryData();

    drive = new Drive(gyroIO, flModuleIO, frModuleIO, blModuleIO, brModuleIO);
    controller = new MockCommandXboxController(0);
  }

  private void setDefaultOdometryData() {
    double[] timestamps = {0.0};
    double[] drivePositions = {0.0};
    Rotation2d[] turnPositions = {new Rotation2d()};

    flModuleIO.setOdometryData(timestamps, drivePositions, turnPositions);
    frModuleIO.setOdometryData(timestamps, drivePositions, turnPositions);
    blModuleIO.setOdometryData(timestamps, drivePositions, turnPositions);
    brModuleIO.setOdometryData(timestamps, drivePositions, turnPositions);

    gyroIO.setOdometryData(timestamps, new Rotation2d[] {new Rotation2d()});
  }

  @Test
  void testJoystickDriveZeroInput() {
    Command command = new TeleopDriveCommand(drive, controller);

    // Initialize and execute the command
    command.initialize();
    command.execute();

    // With zero input, modules should receive zero or near-zero velocity commands
    // (After deadband and squaring, 0.0 should remain 0.0)
    assertEquals(0.0, flModuleIO.lastDriveVelocitySetpoint, DELTA);
    assertEquals(0.0, frModuleIO.lastDriveVelocitySetpoint, DELTA);
    assertEquals(0.0, blModuleIO.lastDriveVelocitySetpoint, DELTA);
    assertEquals(0.0, brModuleIO.lastDriveVelocitySetpoint, DELTA);
  }

  @Test
  void testBumpZoneAngle() {
    assertTrue(
        TeleopDriveCommand.getBumpLockAngle(Rotation2d.fromDegrees(5.0)).getDegrees() == 45.0);
    assertTrue(
        TeleopDriveCommand.getBumpLockAngle(Rotation2d.fromDegrees(-5.0)).getDegrees() == -45.0);
    assertTrue(
        TeleopDriveCommand.getBumpLockAngle(Rotation2d.fromDegrees(175.0)).getDegrees() == 135.0);
    assertTrue(
        TeleopDriveCommand.getBumpLockAngle(Rotation2d.fromDegrees(-175.0)).getDegrees() == -135.0);
  }

  @Test
  void testTrenchZoneAngle() {
    assertTrue(
        TeleopDriveCommand.getTrenchLockAngle(Rotation2d.fromDegrees(100.0)).getDegrees() == 180.0);
    assertTrue(
        TeleopDriveCommand.getTrenchLockAngle(Rotation2d.fromDegrees(-100.0)).getDegrees()
            == -180.0);
    assertTrue(
        TeleopDriveCommand.getTrenchLockAngle(Rotation2d.fromDegrees(80.0)).getDegrees() == 0.0);
    assertTrue(
        TeleopDriveCommand.getTrenchLockAngle(Rotation2d.fromDegrees(-80.0)).getDegrees() == 0.0);
  }

  @Test
  void testBumpTrigger() {
    TeleopDriveCommand teleopDrive = new TeleopDriveCommand(drive, null);

    // Left bump zone - Current Alliance
    drive.setPose(new Pose2d(4.0, 5.5, new Rotation2d()));
    assertTrue(teleopDrive.inBumpZone());

    // Right bump zone - Current Alliance
    drive.setPose(new Pose2d(4.0, 2.5, new Rotation2d()));
    assertTrue(teleopDrive.inBumpZone());

    // Left bump zone - Opposite Alliance
    drive.setPose(new Pose2d(12.0, 5.5, new Rotation2d()));
    assertTrue(teleopDrive.inBumpZone());

    // Right bump zone - Opposite Alliance
    drive.setPose(new Pose2d(12.0, 2.5, new Rotation2d()));
    assertTrue(teleopDrive.inBumpZone());

    // Outside bump zones
    drive.setPose(new Pose2d(8.0, 5.5, new Rotation2d()));
    assertTrue(!teleopDrive.inBumpZone());
  }

  @Test
  void testTrenchTrigger() {
    TeleopDriveCommand teleopDrive = new TeleopDriveCommand(drive, null);

    // Left trench zone - Current Alliance
    drive.setPose(new Pose2d(3.5, 7.5, new Rotation2d()));
    assertTrue(teleopDrive.inTrenchZone());

    // Right trench zone - Current Alliance
    drive.setPose(new Pose2d(4.50, .75, new Rotation2d()));
    assertTrue(teleopDrive.inTrenchZone());

    // Left trench zone - Opposite Alliance
    drive.setPose(new Pose2d(12.0, 7.5, new Rotation2d()));
    assertTrue(teleopDrive.inTrenchZone());

    // Right trench zone - Opposite Alliance
    drive.setPose(new Pose2d(12.0, 0.75, new Rotation2d()));
    assertTrue(teleopDrive.inTrenchZone());

    // Outside trench zones
    drive.setPose(new Pose2d(8.0, 5.5, new Rotation2d()));
    assertTrue(!teleopDrive.inTrenchZone());
  }
}
