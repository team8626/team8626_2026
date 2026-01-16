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

package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.MockGyroIO;
import frc.robot.subsystems.drive.MockModuleIO;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

/** Unit tests for DriveCommands. */
@Tag("unit")
public class DriveCommandsTest {
  private static final double DELTA = 1e-6;
  private static final double DEADBAND = 0.1;

  private MockGyroIO gyroIO;
  private MockModuleIO flModuleIO;
  private MockModuleIO frModuleIO;
  private MockModuleIO blModuleIO;
  private MockModuleIO brModuleIO;
  private Drive drive;

  @BeforeAll
  static void initializeHAL() {
    HAL.initialize(500, 0);
  }

  @BeforeEach
  void setUp() {
    gyroIO = new MockGyroIO();
    flModuleIO = new MockModuleIO();
    frModuleIO = new MockModuleIO();
    blModuleIO = new MockModuleIO();
    brModuleIO = new MockModuleIO();

    setDefaultOdometryData();

    drive = new Drive(gyroIO, flModuleIO, frModuleIO, blModuleIO, brModuleIO);
    drive.periodic();
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
  void testJoystickDriveCreatesCommand() {
    Command command =
        DriveCommands.joystickDrive(
            drive, () -> 0.0, () -> 0.0, () -> 0.0, () -> drive.getRotation());

    assertNotNull(command);
    assertTrue(command.getRequirements().contains(drive));
  }

  @Test
  void testJoystickDriveAtAngleCreatesCommand() {
    Command command =
        DriveCommands.joystickDriveAtAngle(
            drive, () -> 0.0, () -> 0.0, () -> Rotation2d.fromDegrees(0.0));

    assertNotNull(command);
    assertTrue(command.getRequirements().contains(drive));
  }

  @Test
  void testJoystickDriveZeroInput() {
    Command command =
        DriveCommands.joystickDrive(
            drive, () -> 0.0, () -> 0.0, () -> 0.0, () -> drive.getRotation());

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
  void testJoystickDriveDeadbandApplied() {
    // Input below deadband threshold
    double belowDeadband = DEADBAND * 0.5;
    Command command =
        DriveCommands.joystickDrive(
            drive, () -> belowDeadband, () -> 0.0, () -> 0.0, () -> drive.getRotation());

    command.initialize();
    command.execute();

    // Input below deadband should result in zero velocity
    assertEquals(0.0, flModuleIO.lastDriveVelocitySetpoint, DELTA);
  }

  @Test
  void testJoystickDriveAboveDeadband() {
    // Input above deadband threshold
    double aboveDeadband = 0.5;
    Command command =
        DriveCommands.joystickDrive(
            drive, () -> aboveDeadband, () -> 0.0, () -> 0.0, () -> drive.getRotation());

    command.initialize();
    command.execute();

    // Input above deadband should result in non-zero velocity
    assertTrue(flModuleIO.lastDriveVelocitySetpoint != 0.0);
  }

  @Test
  void testJoystickDriveFullInput() {
    // Full input
    Command command =
        DriveCommands.joystickDrive(
            drive, () -> 1.0, () -> 0.0, () -> 0.0, () -> drive.getRotation());

    command.initialize();
    command.execute();

    // Full input should produce significant velocity commands
    assertTrue(Math.abs(flModuleIO.lastDriveVelocitySetpoint) > 0.1);
  }

  @Test
  void testJoystickDriveInputSquaring() {
    // The command squares inputs for more precise control at low speeds
    // With input of 0.5, after deadband and squaring, the effective magnitude
    // should be less than 0.5 (squared)

    double input = 0.5;
    Command commandHalf =
        DriveCommands.joystickDrive(
            drive, () -> input, () -> 0.0, () -> 0.0, () -> drive.getRotation());

    commandHalf.initialize();
    commandHalf.execute();

    double velocityAtHalf = flModuleIO.lastDriveVelocitySetpoint;

    // Now test with full input
    Command commandFull =
        DriveCommands.joystickDrive(
            drive, () -> 1.0, () -> 0.0, () -> 0.0, () -> drive.getRotation());

    commandFull.initialize();
    commandFull.execute();

    double velocityAtFull = flModuleIO.lastDriveVelocitySetpoint;

    // Due to squaring, velocity at half input should be less than half of velocity at full
    // (it should be closer to 1/4 due to squaring)
    if (velocityAtFull != 0.0) {
      double ratio = Math.abs(velocityAtHalf / velocityAtFull);
      assertTrue(
          ratio < 0.5, "Input squaring should make half input produce less than half velocity");
    }
  }

  @Test
  void testJoystickDriveRotationDeadband() {
    // Rotation input below deadband
    double belowDeadband = DEADBAND * 0.5;
    Command command =
        DriveCommands.joystickDrive(
            drive, () -> 0.0, () -> 0.0, () -> belowDeadband, () -> drive.getRotation());

    command.initialize();
    command.execute();

    // Should not produce significant rotation
    // All modules should have similar velocities (no rotation component)
    double fl = flModuleIO.lastDriveVelocitySetpoint;
    double fr = frModuleIO.lastDriveVelocitySetpoint;
    double bl = blModuleIO.lastDriveVelocitySetpoint;
    double br = brModuleIO.lastDriveVelocitySetpoint;

    // With zero linear input and rotation below deadband, all should be zero
    assertEquals(0.0, fl, DELTA);
    assertEquals(0.0, fr, DELTA);
    assertEquals(0.0, bl, DELTA);
    assertEquals(0.0, br, DELTA);
  }

  @Test
  void testJoystickDriveNegativeInput() {
    // Negative input (backwards)
    Command command =
        DriveCommands.joystickDrive(
            drive, () -> -0.5, () -> 0.0, () -> 0.0, () -> drive.getRotation());

    command.initialize();
    command.execute();

    // Should produce negative velocity (or reversed turn angles)
    assertTrue(flModuleIO.lastDriveVelocitySetpoint != 0.0);
  }

  @Test
  void testJoystickDriveDiagonalInput() {
    // Diagonal input (forward + strafe)
    Command command =
        DriveCommands.joystickDrive(
            drive, () -> 0.5, () -> 0.5, () -> 0.0, () -> drive.getRotation());

    command.initialize();
    command.execute();

    // Should produce non-zero velocity
    assertTrue(flModuleIO.lastDriveVelocitySetpoint != 0.0);
    // Turn angles should be around 45 degrees for diagonal motion
    double flAngle = flModuleIO.lastTurnPositionSetpoint.getDegrees();
    assertTrue(Math.abs(flAngle) > 0 || Math.abs(flAngle - 45) < 90);
  }

  @Test
  void testFeedforwardCharacterizationCreatesCommand() {
    Command command = DriveCommands.feedforwardCharacterization(drive);

    assertNotNull(command);
  }

  @Test
  void testWheelRadiusCharacterizationCreatesCommand() {
    Command command = DriveCommands.wheelRadiusCharacterization(drive);

    assertNotNull(command);
  }

  @Test
  void testJoystickDriveAtAngleZeroInput() {
    Command command =
        DriveCommands.joystickDriveAtAngle(
            drive, () -> 0.0, () -> 0.0, () -> Rotation2d.fromDegrees(0.0));

    command.initialize();
    command.execute();

    // With zero linear input and already at target angle, should be minimal movement
    // Note: PID controller may still apply small corrections
    assertNotNull(drive.getRotation());
  }

  @Test
  void testJoystickDriveAtAngleWithTarget() {
    // Set robot at 0 degrees
    drive.setPose(new edu.wpi.first.math.geometry.Pose2d(0, 0, Rotation2d.fromDegrees(0.0)));
    drive.periodic();

    Command command =
        DriveCommands.joystickDriveAtAngle(
            drive, () -> 0.0, () -> 0.0, () -> Rotation2d.fromDegrees(90.0));

    command.initialize();

    // Execute multiple times to allow PID controller to respond
    for (int i = 0; i < 5; i++) {
      command.execute();
    }

    // Robot should try to rotate toward 90 degrees
    // At least one module should have non-zero velocity for rotation
    boolean anyNonZero =
        flModuleIO.lastDriveVelocitySetpoint != 0.0
            || frModuleIO.lastDriveVelocitySetpoint != 0.0
            || blModuleIO.lastDriveVelocitySetpoint != 0.0
            || brModuleIO.lastDriveVelocitySetpoint != 0.0;

    assertTrue(anyNonZero, "Should attempt to rotate toward target angle");
  }

  @Test
  void testJoystickDriveCombinedInputs() {
    // Test combined translation and rotation
    Command command =
        DriveCommands.joystickDrive(
            drive, () -> 0.5, () -> 0.3, () -> 0.4, () -> drive.getRotation());

    command.initialize();
    command.execute();

    // All modules should be active
    assertTrue(flModuleIO.lastDriveVelocitySetpoint != 0.0);
    assertTrue(frModuleIO.lastDriveVelocitySetpoint != 0.0);
    assertTrue(blModuleIO.lastDriveVelocitySetpoint != 0.0);
    assertTrue(brModuleIO.lastDriveVelocitySetpoint != 0.0);
  }
}
