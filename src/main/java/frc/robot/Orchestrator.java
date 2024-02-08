package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeNote.IntakeNote;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.pixy2.Pixy2;
import frc.robot.subsystems.shooter.Shooter;

public class Orchestrator {
  private final Drive drive;
  private final IntakeNote intakeNote;
  private final Indexer indexer;
  private final Shooter shooter;
  private final Pixy2 pixy2;
  private final Arm arm;

  public Orchestrator(
      Drive drive, IntakeNote intakeNote, Indexer indexer, Shooter shooter, Pixy2 pixy2, Arm arm) {
    this.drive = drive;
    this.intakeNote = intakeNote;
    this.indexer = indexer;
    this.shooter = shooter;
    this.pixy2 = pixy2;
    this.arm = arm;
  }

  public Command shootBasic() {
    return Commands.sequence(
            Commands.parallel(
                shooter.shoot(9999)
                // Turn towards speaker.
                ),
            Commands.waitUntil(shooter::ShooterSpeedIsReady).withTimeout(2),
            indexer.setIndexerPercentVoltage(1),
            Commands.waitUntil(() -> !indexer.getLimitSwitchPressed()).withTimeout(2),
            Commands.parallel(indexer.setIndexerPercentVoltage(0), shooter.manualShoot(0)))
        .onlyIf(indexer::getLimitSwitchPressed);
  }

  public Command intakeBasic() {
    return Commands.sequence(
            indexer.setIndexerPercentVoltage(4),
            arm.toSetpoint(new Rotation2d()), //TODO Make setPoint for pickup position.
            Commands.waitUntil(arm::atSetpoint).withTimeout(2),
            intakeNote.intake());
  }

  public Command expel() {
    return Commands.parallel(
            // arm.toSetpoint(new Rotation2d()), //TODO Make setPoint for pickup position.
            // Commands.waitUntil(arm::atSetpoint).withTimeout(2),
            shooter.manualShoot(-5),
            indexer.setIndexerVoltage(-5.0),
            intakeNote.release());
  }
}
