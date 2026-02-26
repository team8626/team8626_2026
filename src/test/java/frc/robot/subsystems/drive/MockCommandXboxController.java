package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class MockCommandXboxController extends CommandXboxController {
  public double LeftX = 0.0;
  public double LeftY = 0.0;
  public double RightX = 0.0;
  public double RightY = 0.0;

  public MockCommandXboxController(int port) {
    super(port);
  }

  @Override
  public double getLeftX() {
    return LeftX;
  }

  @Override
  public double getLeftY() {
    return LeftY;
  }

  @Override
  public double getRightX() {
    return RightX;
  }

  @Override
  public double getRightY() {
    return RightY;
  }
}
