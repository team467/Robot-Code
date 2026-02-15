package frc.robot;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopperbelt.HopperBelt;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class Orchestrator {
  private final Drive drive;
  private final Shooter shooter;
  private final HopperBelt hopperBelt;
  private final Indexer indexer;
  private final Intake intake;
  private final RobotState robotState = RobotState.getInstance();

  public Orchestrator(
      Drive drive, HopperBelt hopperBelt, Shooter shooter, Indexer indexer, Intake intake) {
    this.drive = drive;
    this.hopperBelt = hopperBelt;
    this.shooter = shooter;
    this.indexer = indexer;
    this.intake = intake;
  }
}
