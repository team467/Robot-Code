package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.utils.AllianceFlipUtil;
import frc.robot.FieldConstants;
import frc.robot.Orchestrator;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class Autos {
  private final Drive drive;
  private final Arm arm;
  private final Orchestrator orchestrator;

  private double MOBILITY_DRIVE_DISTANCE = Units.feetToMeters(6.75);

  @AutoLogOutput(key = "Autos/Notes/0")
  private Translation2d noteTranslation;

  @AutoLogOutput(key = "Autos/Notes/1")
  private Translation2d secondNoteTranslation;

  @AutoLogOutput(key = "Autos/Notes/2")
  private Translation2d thirdNoteTranslation;

  public Autos(Drive drive, Arm arm, Orchestrator orchestrator) {
    this.drive = drive;
    this.arm = arm;
    this.orchestrator = orchestrator;
  }

  public enum StartingPosition {
    LEFT,
    CENTER,
    RIGHT;

    public Pose2d getStartingPosition() {
      return getRelativeStartingPosition().getStartingPosition();
    }

    RelativeStartingPosition getRelativeStartingPosition() {
      switch (this) {
        case LEFT:
          return AllianceFlipUtil.shouldFlip()
              ? RelativeStartingPosition.NEAR_SOURCE
              : RelativeStartingPosition.NEAR_AMP;
        case CENTER:
          return RelativeStartingPosition.CENTER;

        case RIGHT:
          return AllianceFlipUtil.shouldFlip()
              ? RelativeStartingPosition.NEAR_AMP
              : RelativeStartingPosition.NEAR_SOURCE;
      }
      throw new RuntimeException("Invalid position");
    }
  }

  private enum RelativeStartingPosition {
    NEAR_AMP,
    CENTER,
    NEAR_SOURCE;

    public Pose2d getStartingPosition() {
      switch (this) {
        case NEAR_AMP:
          return AllianceFlipUtil.apply(FieldConstants.Subwoofer.ampFaceCorner);
        case CENTER:
          return AllianceFlipUtil.apply(FieldConstants.Subwoofer.centerFace);
        case NEAR_SOURCE:
          return AllianceFlipUtil.apply(FieldConstants.Subwoofer.sourceFaceCorner);
      }
      throw new RuntimeException("Invalid position");
    }
  }

  private Command setNotePositions(Supplier<StartingPosition> position) {
    return Commands.runOnce(
            () -> {
              switch (position.get()) {
                case RIGHT -> {
                  this.noteTranslation = getNotePositions(0, false);
                  this.secondNoteTranslation = getNotePositions(1, false);
                  this.thirdNoteTranslation = getNotePositions(2, false);
                }
                case CENTER -> {
                  this.noteTranslation = getNotePositions(1, false);
                  this.secondNoteTranslation = getNotePositions(2, false);
                  this.thirdNoteTranslation = getNotePositions(0, false);
                }
                case LEFT -> {
                  this.noteTranslation = getNotePositions(2, false);
                  this.secondNoteTranslation = getNotePositions(1, false);
                  this.thirdNoteTranslation = getNotePositions(0, false);
                }
              }
            })
        .until(() -> true);
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
    return Commands.runOnce(() -> drive.setPose(position.getStartingPosition()))
        .andThen(
            Commands.select(
                Map.of(
                    RelativeStartingPosition.NEAR_SOURCE,
                    driveXDistance(MOBILITY_DRIVE_DISTANCE).withTimeout(5),
                    RelativeStartingPosition.CENTER,
                    driveXDistance(MOBILITY_DRIVE_DISTANCE).withTimeout(5),
                    RelativeStartingPosition.NEAR_AMP,
                    Commands.run(
                            () -> drive.runVelocity(new ChassisSpeeds(Units.feetToMeters(9), 0, 0)))
                        .withTimeout(1)
                        .andThen(driveXDistance(MOBILITY_DRIVE_DISTANCE).withTimeout(5))),
                position::getRelativeStartingPosition));
  }

  private Command driveXDistance(double distance) {
    return Commands.defer(
        () -> new StraightDriveToPose(AllianceFlipUtil.applyRelative(distance), 0, 0, drive),
        Set.of(drive));
  }

  public Command oneNoteAuto() {
    return arm.runPercent(-0.3)
        .until(arm::limitSwitchPressed)
        .withTimeout(1)
        .andThen(arm.toSetpoint(ArmConstants.AFTER_INTAKE_POS).withTimeout(1))
        .andThen(orchestrator.shootBasic().withTimeout(3));
  }

  public Command scoreOneNoteMobility(StartingPosition position) {
    return oneNoteAuto().andThen(mobilityAuto(position));
  }

  public Command twoNoteAuto(StartingPosition position) {
    return setNotePositions(() -> position)
        .andThen(
            oneNoteAuto()
                .andThen(
                    Commands.parallel(
                        orchestrator.driveToNote(() -> noteTranslation).withTimeout(5),
                        orchestrator.intakeBasic()))
                .andThen(shoot()));
  }

  public Command noVisionFourNoteAuto() {
    StartingPosition position = StartingPosition.CENTER;
    return Commands.parallel(
            Commands.runOnce(() -> drive.setPose(position.getStartingPosition())),
            setNotePositions(() -> position))
        .andThen(
            oneNoteAuto()
                .andThen(scoreCycle(() -> noteTranslation, position::getStartingPosition))
                .andThen(scoreCycle(() -> secondNoteTranslation, position::getStartingPosition))
                .andThen(
                    stageNoteCycle(() -> thirdNoteTranslation, position::getStartingPosition)));
  }

  public Command threeNoteAuto(StartingPosition position) {
    return setNotePositions(() -> position)
        .andThen(
            oneNoteAuto()
                .andThen(
                    Commands.parallel(
                        orchestrator.driveToNote(() -> noteTranslation),
                        orchestrator.intakeBasic()))
                .andThen(shoot())
                .andThen(
                    Commands.parallel(
                            orchestrator.driveToNote(() -> secondNoteTranslation),
                            orchestrator.intakeBasic())
                        .andThen(shoot())));
  }

  private Command shoot() {
    return orchestrator.shootBasic();
  }

  public Command fourNoteAuto(StartingPosition position) {
    return setNotePositions(() -> position)
        .andThen(
            oneNoteAuto()
                .withTimeout(1)
                .andThen(
                    Commands.parallel(
                        orchestrator.driveToNote(() -> noteTranslation),
                        orchestrator.intakeBasic()))
                .andThen(shoot())
                .andThen(
                    Commands.parallel(
                            orchestrator.driveToNote(() -> secondNoteTranslation),
                            orchestrator.intakeBasic())
                        .andThen(shoot()))
                .andThen(
                    Commands.parallel(
                        orchestrator.driveToNote(() -> thirdNoteTranslation),
                        orchestrator.intakeBasic()))
                .andThen(shoot()));
  }

  private Command scoreCycle(
      Supplier<Translation2d> noteTranslation, Supplier<Pose2d> shootPosition) {
    return Commands.race(orchestrator.driveToNote(noteTranslation), orchestrator.intakeBasic())
        .andThen(orchestrator.deferredStraightDriveToPose(shootPosition).withTimeout(4))
        .andThen(shoot().withTimeout(4));
  }

  private Command stageNoteCycle(
      Supplier<Translation2d> noteTranslation, Supplier<Pose2d> shootPosition) {
    return Commands.race(
            orchestrator
                .deferredStraightDriveToPose(
                    () ->
                        new Pose2d(
                            drive.getPose().getTranslation().getX(),
                            noteTranslation.get().getY()
                                + AllianceFlipUtil.applyRelative(Units.inchesToMeters(-5)),
                            new Rotation2d()))
                .andThen(
                    orchestrator.deferredStraightDriveToPose(
                        () ->
                            new Pose2d(
                                noteTranslation.get().getX(),
                                drive.getPose().getTranslation().getY(),
                                new Rotation2d())))
                .withTimeout(3),
            orchestrator.intakeBasic())
        .withTimeout(5)
        .andThen(
            orchestrator
                .deferredStraightDriveToPose(shootPosition)
                .withTimeout(3)
                .alongWith(shoot().withTimeout(5)));
  }
}
