package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.lib.utils.AllianceFlipUtil;
import frc.robot.FieldConstants;
import frc.robot.Orchestrator;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pixy2.Pixy2;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Autos {
  private final Drive drive;
  private final Orchestrator orchestrator;
  private Translation2d noteTranslation;
  private Translation2d secondNoteTranslation;
  private Translation2d thirdNoteTranslation;

  public Autos(Drive drive, Orchestrator orchestrator) {
    this.drive = drive;
    this.orchestrator = orchestrator;
  }

  public enum StartingPosition {
    LEFT,
    CENTER,
    RIGHT
  }

  private void setNotePositions(StartingPosition position) {
    switch (position) {
      case LEFT -> {
        this.noteTranslation =
            AllianceFlipUtil.apply(FieldConstants.StagingLocations.spikeTranslations[0]);
        this.secondNoteTranslation =
            AllianceFlipUtil.apply(FieldConstants.StagingLocations.spikeTranslations[1]);
        this.thirdNoteTranslation =
            AllianceFlipUtil.apply(FieldConstants.StagingLocations.spikeTranslations[2]);
      }
      case CENTER -> {
        this.noteTranslation =
            AllianceFlipUtil.apply(FieldConstants.StagingLocations.spikeTranslations[1]);
        this.secondNoteTranslation =
            AllianceFlipUtil.apply(FieldConstants.StagingLocations.spikeTranslations[2]);
        this.thirdNoteTranslation =
            AllianceFlipUtil.apply(FieldConstants.StagingLocations.spikeTranslations[0]);
      }
      case RIGHT -> {
        this.noteTranslation =
            AllianceFlipUtil.apply(FieldConstants.StagingLocations.spikeTranslations[2]);
        this.secondNoteTranslation =
            AllianceFlipUtil.apply(FieldConstants.StagingLocations.spikeTranslations[1]);
        this.thirdNoteTranslation =
            AllianceFlipUtil.apply(FieldConstants.StagingLocations.spikeTranslations[0]);
      }
    }

    Logger.recordOutput("Autos/NotePositions/0", noteTranslation);
    Logger.recordOutput("Autos/NotePositions/1", secondNoteTranslation);
    Logger.recordOutput("Autos/NotePositions/2", thirdNoteTranslation);
  }


  public Command oneNoteAuto() {
    return orchestrator.fullAlignShootSpeaker();
  }

  public Command twoNoteAuto(StartingPosition position) {
    setNotePositions(position);
    return orchestrator.fullAlignShootSpeaker()
            .andThen(
            Commands.parallel(
                    orchestrator.driveToNote(noteTranslation),
                    orchestrator.intakeBasic()))
        .andThen(orchestrator.fullAlignShootSpeaker());
  }

  public Command threeNoteAuto(StartingPosition position) {
    setNotePositions(position);
    return orchestrator.fullAlignShootSpeaker()
            .andThen(
                    Commands.parallel(
                            orchestrator.driveToNote(noteTranslation),
                            orchestrator.intakeBasic()))
            .andThen(orchestrator.fullAlignShootSpeaker())
        .andThen(
            Commands.parallel(
                            orchestrator.driveToNote(secondNoteTranslation),
                    orchestrator.intakeBasic())
                .andThen(orchestrator.fullAlignShootSpeaker()));
  }

  public Command fourNoteAuto(StartingPosition position) {
    setNotePositions(position);
    return orchestrator.fullAlignShootSpeaker()
            .andThen(
                    Commands.parallel(
                            orchestrator.driveToNote(noteTranslation),
                            orchestrator.intakeBasic()))
            .andThen(orchestrator.fullAlignShootSpeaker())
            .andThen(
                    Commands.parallel(
                                    orchestrator.driveToNote(secondNoteTranslation),
                                    orchestrator.intakeBasic())
                            .andThen(orchestrator.fullAlignShootSpeaker()))
        .andThen(
            Commands.parallel(
                            orchestrator.driveToNote(thirdNoteTranslation),orchestrator.intakeBasic()
                    )).andThen(orchestrator.fullAlignShootSpeaker());
  }
}
