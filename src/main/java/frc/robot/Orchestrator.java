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

  public Command shootAndIndex(double shooterTargetRPM) {
    return Commands.parallel(
        indexer
            .run()
            .until(
                () ->
                    !shooter.isAtSetpoint()
                        && (indexer.isLeftSwitchPressed() | indexer.isRightSwitchPressed())),
        shooter.setTargetVelocity(shooterTargetRPM));
  }
  ;
}
