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
    private final Pose2d startingPosition;

    StartingPosition(Pose2d startingPosition) {
      this.startingPosition = startingPosition;
      Logger.recordOutput("Autos/StartingPositions/" + this, startingPosition);
    }

    public Pose2d getStartingPosition() {
      return startingPosition;
    }
  };

  private Command setNotePositions(StartingPosition position) {
    return Commands.runOnce(
        () -> {
          if (Constants.getRobot() == Constants.RobotType.ROBOT_SIMBOT) {
            drive.setPose(position.getStartingPosition());
          }
          switch (position) {
            case LEFT -> {
              this.noteTranslation = getNotePositions(0, false);
              this.secondNoteTranslation = getNotePositions(1, false);
              this.thirdNoteTranslation = getNotePositions(2, true);
            }
            case CENTER -> {
              this.noteTranslation = getNotePositions(1, false);
              this.secondNoteTranslation = getNotePositions(2, false);
              this.thirdNoteTranslation = getNotePositions(0, true);
            }
            case RIGHT -> {
              this.noteTranslation = getNotePositions(2, false);
              this.secondNoteTranslation = getNotePositions(1, false);
              this.thirdNoteTranslation = getNotePositions(0, true);
            }
          }

          Logger.recordOutput("Autos/NotePositions/0", noteTranslation);
          Logger.recordOutput("Autos/NotePositions/1", secondNoteTranslation);
          Logger.recordOutput("Autos/NotePositions/2", thirdNoteTranslation);
        });
  }

  private Translation2d getNotePositions(int index, boolean centerNotes) {
    return AllianceFlipUtil.apply(
        centerNotes
            ? FieldConstants.StagingLocations.centerlineTranslations[
                AllianceFlipUtil.shouldFlip() ? 4 - index : index]
            : FieldConstants.StagingLocations.spikeTranslations[
                AllianceFlipUtil.shouldFlip() ? 2 - index : index]);
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
    return shoot();
  }

  public Command scoreOneNoteMobility(StartingPosition position) {
    return Commands.sequence(oneNoteAuto().andThen(mobilityAuto(position)));
  }

  public Command twoNoteAuto(StartingPosition position) {
    return setNotePositions(position)
        .andThen(
            orchestrator
                .fullAlignShootSpeaker()
                .andThen(
                    Commands.parallel(
                        orchestrator.driveToNote(noteTranslation), orchestrator.intakeBasic()))
                .andThen(shoot()));
  }

  public Command threeNoteAuto(StartingPosition position) {
    return setNotePositions(position)
        .andThen(
            orchestrator
                .fullAlignShootSpeaker()
                .andThen(
                    Commands.parallel(
                        orchestrator.driveToNote(noteTranslation), orchestrator.intakeBasic()))
                .andThen(shoot())
                .andThen(
                    Commands.parallel(
                            orchestrator.driveToNote(secondNoteTranslation),
                            orchestrator.intakeBasic())
                        .andThen(shoot())));
  }

  private Command shoot() {
    return Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0, 0, 0)))
        .onlyWhile(
            () ->
                drive
                        .getPose()
                        .getTranslation()
                        .getDistance(
                            AllianceFlipUtil.apply(
                                FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d()))
                    > Units.inchesToMeters(30))
        .andThen(orchestrator.fullAlignShootSpeaker());
  }

  public Command fourNoteAuto(StartingPosition position) {
    return setNotePositions(position)
        .andThen(
            shoot()
                .andThen(
                    Commands.parallel(
                        orchestrator.driveToNote(noteTranslation), orchestrator.intakeBasic()))
                .andThen(shoot())
                .andThen(
                    Commands.parallel(
                            orchestrator.driveToNote(secondNoteTranslation),
                            orchestrator.intakeBasic())
                        .andThen(shoot()))
                .andThen(
                    Commands.parallel(
                        orchestrator.driveToNote(thirdNoteTranslation), orchestrator.intakeBasic()))
                .andThen(shoot()));
  }
}
