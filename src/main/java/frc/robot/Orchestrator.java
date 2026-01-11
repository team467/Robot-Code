package frc.robot;

import frc.robot.subsystems.drive.Drive;

public class Orchestrator {
  private final Drive drive;
  private final RobotState robotState = RobotState.getInstance();

  public Orchestrator(Drive drive) {
    this.drive = drive;
  }
}
