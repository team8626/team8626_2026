package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.indexer.IndexerIOSim;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

/** Unit tests for the Drive subsystem. */
@Tag("unit")
public class SpindexerCommandsTest {
  private static final double DELTA = 1e-3;

  private IndexerIOSim io;
  private Indexer spindexer;

  @BeforeAll
  static void initializeHAL() {
    HAL.initialize(500, 0);
  }

  @BeforeEach
  void setUp() {
    io = new IndexerIOSim();
    spindexer = new Indexer(io);
  }

  @Test
  void testIndexerStartCommand() {
    IndexerStartCommand command = new IndexerStartCommand(spindexer);

    // Check if the desired velocity is set correctly
    command.initialize();
    spindexer.periodic();
    assertTrue(spindexer.getDesiredVelocity().equals(IndexerConstants.DEFAULT_VELOCITY));

    // Check if the adjusting speed on the fly
    command.setTargetVelocity(() -> RPM.of(123.0));
    spindexer.periodic();
    assertTrue(spindexer.getDesiredVelocity().equals(RPM.of(123.0)));

    // Check if the Spindexer is stopped on end()
    command.end(false);
    spindexer.periodic();
    assertTrue(spindexer.getDesiredVelocity().equals(RPM.of(0.0)));
  }

  @Test
  void testIndexerStartCommandWithVelocity() {
    AngularVelocity targetVelocity = RPM.of(123.0);
    IndexerStartCommand command = new IndexerStartCommand(() -> targetVelocity, spindexer);

    // Check if the desired velocity is set correctly
    command.initialize();
    spindexer.periodic();
    assertTrue(spindexer.getDesiredVelocity().equals(targetVelocity));
  }

  @Test
  void testIndexerStopCommand() {
    IndexerStartCommand startCommand = new IndexerStartCommand(spindexer);
    IndexerStopCommand stopCommand = new IndexerStopCommand(spindexer);

    // Check if the desired velocity is set correctly
    startCommand.initialize();
    spindexer.periodic();
    assertTrue(spindexer.getDesiredVelocity().equals(IndexerConstants.DEFAULT_VELOCITY));

    // Test the stop command, desiredVelocity should be 0
    stopCommand.initialize();
    spindexer.periodic();
    assertTrue(spindexer.getDesiredVelocity().equals(RPM.of(0.0)));
  }
}
