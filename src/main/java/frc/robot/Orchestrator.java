package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.magiccarpet.MagicCarpet;
import frc.robot.subsystems.shooter.Shooter;

public class Orchestrator {
  private final Drive drive;
  private final Shooter shooter;
  private final MagicCarpet magicCarpet;
  private final Indexer indexer;
  private final Intake intake;
  private final RobotState robotState = RobotState.getInstance();

  public Orchestrator(
      Drive drive, MagicCarpet magicCarpet, Shooter shooter, Indexer indexer, Intake intake) {
    this.drive = drive;
    this.magicCarpet = magicCarpet;
    this.shooter = shooter;
    this.indexer = indexer;
    this.intake = intake;
  }

  /** created command to shoot the balls so it runs the shooter, magicCarpet and indexer */
  public Command shootBalls() {
    return Commands.parallel(shooter.setVoltage(1), magicCarpet.start(), indexer.run());
  }
}
