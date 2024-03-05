package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.utils.AllianceFlipUtil;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.Orchestrator;
import frc.robot.subsystems.drive.Drive;
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
    LEFT(
        AllianceFlipUtil.shouldFlip()
            ? AllianceFlipUtil.apply(FieldConstants.Subwoofer.sourceFaceCorner)
            : AllianceFlipUtil.apply(FieldConstants.Subwoofer.ampFaceCorner)),
    CENTER(AllianceFlipUtil.apply(FieldConstants.Subwoofer.centerFace)),
    RIGHT(
        AllianceFlipUtil.shouldFlip()
            ? AllianceFlipUtil.apply(FieldConstants.Subwoofer.ampFaceCorner)
            : AllianceFlipUtil.apply(FieldConstants.Subwoofer.sourceFaceCorner));
    public final Pose2d startingPosition;

    private StartingPosition(Pose2d startingPosition) {
      this.startingPosition = startingPosition;
    }

    public Pose2d getStartingPosition() {
      return startingPosition;
    }
  };

  private void setNotePositions(StartingPosition position) {
    if (Constants.getRobot() == Constants.RobotType.ROBOT_SIMBOT || 1 == 1) {
      drive.setPose(position.getStartingPosition());
    }
    switch (position) {
      case LEFT -> {
        this.noteTranslation =
            AllianceFlipUtil.apply(
                FieldConstants.StagingLocations.spikeTranslations[
                    AllianceFlipUtil.shouldFlip() ? 2 : 0]);
        this.secondNoteTranslation =
            AllianceFlipUtil.apply(FieldConstants.StagingLocations.spikeTranslations[1]);
        this.thirdNoteTranslation =
            AllianceFlipUtil.apply(
                FieldConstants.StagingLocations.spikeTranslations[
                    AllianceFlipUtil.shouldFlip() ? 0 : 2]);
      }
      case CENTER -> {
        this.noteTranslation =
            AllianceFlipUtil.apply(FieldConstants.StagingLocations.spikeTranslations[1]);
        this.secondNoteTranslation =
            AllianceFlipUtil.apply(
                FieldConstants.StagingLocations.spikeTranslations[
                    AllianceFlipUtil.shouldFlip() ? 0 : 2]);
        this.thirdNoteTranslation =
            AllianceFlipUtil.apply(
                FieldConstants.StagingLocations.spikeTranslations[
                    AllianceFlipUtil.shouldFlip() ? 2 : 0]);
      }
      case RIGHT -> {
        this.noteTranslation =
            AllianceFlipUtil.apply(
                FieldConstants.StagingLocations.spikeTranslations[
                    AllianceFlipUtil.shouldFlip() ? 0 : 2]);
        this.secondNoteTranslation =
            AllianceFlipUtil.apply(FieldConstants.StagingLocations.spikeTranslations[1]);
        this.thirdNoteTranslation =
            AllianceFlipUtil.apply(
                FieldConstants.StagingLocations.spikeTranslations[
                    AllianceFlipUtil.shouldFlip() ? 2 : 0]);
      }
    }

    Logger.recordOutput("Autos/NotePositions/0", noteTranslation);
    Logger.recordOutput("Autos/NotePositions/1", secondNoteTranslation);
    Logger.recordOutput("Autos/NotePositions/2", thirdNoteTranslation);
  }

  public Command mobilityAuto(StartingPosition position) {
    return switch (position) {
      case LEFT, CENTER -> new StraightDriveToPose(Units.feetToMeters(6.75), 0, 0, drive)
          .withTimeout(5);
      case RIGHT -> Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0, 8, 0)))
          .withTimeout(0.125)
          .andThen(new StraightDriveToPose(Units.feetToMeters(6.75), 0, 0, drive).withTimeout(5));
      default -> Commands.none();
    };
  }

  public Command oneNoteAuto() {
    return orchestrator.fullAlignShootSpeaker();
  }

  public Command scoreOneNoteMobility(StartingPosition position) {
    return Commands.sequence(oneNoteAuto().andThen(mobilityAuto(position)));
  }

  public Command twoNoteAuto(StartingPosition position) {
    setNotePositions(position);
    return orchestrator
        .fullAlignShootSpeaker()
        .andThen(
            Commands.parallel(
                orchestrator.driveToNote(noteTranslation), orchestrator.intakeBasic()))
        .andThen(orchestrator.fullAlignShootSpeaker());
  }

  public Command threeNoteAuto(StartingPosition position) {
    setNotePositions(position);
    return orchestrator
        .fullAlignShootSpeaker()
        .andThen(
            Commands.parallel(
                orchestrator.driveToNote(noteTranslation), orchestrator.intakeBasic()))
        .andThen(orchestrator.fullAlignShootSpeaker())
        .andThen(
            Commands.parallel(
                    orchestrator.driveToNote(secondNoteTranslation), orchestrator.intakeBasic())
                .andThen(orchestrator.fullAlignShootSpeaker()));
  }

  public Command fourNoteAuto(StartingPosition position) {
    setNotePositions(position);
    return orchestrator
        .fullAlignShootSpeaker()
        .andThen(
            Commands.parallel(
                orchestrator.driveToNote(noteTranslation), orchestrator.intakeBasic()))
        .andThen(orchestrator.fullAlignShootSpeaker())
        .andThen(
            Commands.parallel(
                    orchestrator.driveToNote(secondNoteTranslation), orchestrator.intakeBasic())
                .andThen(orchestrator.fullAlignShootSpeaker()))
        .andThen(
            Commands.parallel(
                orchestrator.driveToNote(thirdNoteTranslation), orchestrator.intakeBasic()))
        .andThen(orchestrator.fullAlignShootSpeaker());
  }
}
