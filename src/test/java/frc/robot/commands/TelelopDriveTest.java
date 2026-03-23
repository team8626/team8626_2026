package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.AkitDrive;
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
  private AkitDrive drive;
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

    drive = new AkitDrive(gyroIO, flModuleIO, frModuleIO, blModuleIO, brModuleIO);
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
}
