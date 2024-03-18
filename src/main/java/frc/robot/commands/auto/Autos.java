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
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class Autos {
  private final Drive drive;
  private final Arm arm;
  private final Orchestrator orchestrator;

  private double MOBILITY_DRIVE_DISTANCE = Units.feetToMeters(6.75);

  @AutoLogOutput(key = "Autos/Notes/")
  private Translation2d[] noteTranslations = new Translation2d[3];

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
                  this.noteTranslations[0] = getNotePositions(0, false);
                  this.noteTranslations[1] = getNotePositions(1, false);
                  this.noteTranslations[2] = getNotePositions(2, false);
                }
                case CENTER -> {
                       this.noteTranslations[0] = getNotePositions(1, false);
                  this.noteTranslations[1] = getNotePositions(2, false);
                   this.noteTranslations[2] = getNotePositions(0, false);
                }
                case LEFT -> {
                       this.noteTranslations[0] = getNotePositions(2, false);
                  this.noteTranslations[1] = getNotePositions(1, false);
                   this.noteTranslations[2] = getNotePositions(0, false);
                }
              }
            })
        .until(() -> true);
  }

  private Translation2d getNotePositions(int index, boolean centerNotes) {
    Translation2d noteTranslation =
        centerNotes
            ? FieldConstants.StagingLocations.centerlineTranslations[
                AllianceFlipUtil.shouldFlip() ? 4 - index : index]
            : FieldConstants.StagingLocations.spikeTranslations[
                AllianceFlipUtil.shouldFlip() ? 2 - index : index];
    return AllianceFlipUtil.apply(
        new Translation2d(
            noteTranslation.getX() - Units.inchesToMeters(7), noteTranslation.getY()));
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

  public Command noVisionFourNoteAuto() {
    return noVisionInit(() -> StartingPosition.CENTER)
        .andThen(
            oneNoteAuto()
                .andThen(scoreCycle(() -> noteTranslations[0], Rotation2d.fromDegrees(5)))
                .andThen(scoreCycle(() -> noteTranslations[1], Rotation2d.fromDegrees(10)))
                .andThen(stageNoteCycle(() -> noteTranslations[2], Rotation2d.fromDegrees(10))));
  }

  private Command noVisionInit(Supplier<StartingPosition> position) {
    return Commands.parallel(
        Commands.runOnce(() -> drive.setPose(position.get().getStartingPosition()))
            .withTimeout(0.02),
        setNotePositions(position).withTimeout(0.02));
  }

  public Command threeNoteAuto(StartingPosition position) {
    return noVisionInit(() -> position)
        .andThen(oneNoteAuto())
        .andThen(
            scoreCycle(
                () -> noteTranslations[0],
                position::getStartingPosition,
                () -> position != StartingPosition.CENTER))
        .andThen(
            scoreCycle(
                () -> noteTranslations[1],
                position::getStartingPosition,
                () -> position != StartingPosition.CENTER));
  }

  public Command noVisionTwoNoteAuto(StartingPosition position) {
    return noVisionInit(() -> position)
        .andThen(oneNoteAuto())
        .andThen(
            scoreCycle(
                () -> noteTranslations[0],
                position::getStartingPosition,
                () -> position != StartingPosition.CENTER));
  }

  private Command scoreCycle(
      Supplier<Translation2d> intakePosition,
      Supplier<Pose2d> shootPosition,
      BooleanSupplier backUp) {
    return Commands.race(
            Commands.run(() -> drive.runVelocity(new ChassisSpeeds(Units.feetToMeters(4), 0, 0)))
                .withTimeout(0.5)
                .onlyIf(backUp)
                .andThen(orchestrator.driveToNote(intakePosition)),
            orchestrator.intakeBasic())
        .andThen(orchestrator.deferredStraightDriveToPose(shootPosition).withTimeout(4).alongWith(orchestrator.spinUpFlywheel())).andThen(orchestrator.indexBasic().withTimeout(1));
  }

  private Command scoreCycle(Supplier<Translation2d> intakePosition, Rotation2d armAngle) {
    return Commands.race(orchestrator.driveToNote(intakePosition), orchestrator.intakeBasic())
        .andThen(
            Commands.parallel(
                    orchestrator.turnToSpeaker().withTimeout(3),
                    arm.toSetpoint(armAngle),
                    Commands.waitUntil(arm::atSetpoint),
                    Commands.waitSeconds(0.1),
                    Commands.sequence(
                        Commands.waitSeconds(1.5), orchestrator.spinUpFlywheel())).andThen(orchestrator.indexBasic().withTimeout(0.5))
                .withTimeout(3));
  }

  private Command stageNoteCycle(Supplier<Translation2d> intakePosition, Rotation2d armAngle) {
    return Commands.race(
            orchestrator
                .deferredStraightDriveToPose(
                    () ->
                        new Pose2d(
                            drive.getPose().getTranslation().getX()
                                - AllianceFlipUtil.applyRelative(Units.inchesToMeters(20)),
                            intakePosition.get().getY()
                                + AllianceFlipUtil.applyRelative(Units.inchesToMeters(-5)),
                            new Rotation2d()))
                .andThen(
                    orchestrator.deferredStraightDriveToPose(
                        () ->
                            new Pose2d(
                                intakePosition.get().getX()
                                    - AllianceFlipUtil.applyRelative(Units.inchesToMeters(9)),
                                drive.getPose().getTranslation().getY(),
                                new Rotation2d())))
                .withTimeout(3),
            orchestrator.intakeBasic())
        .withTimeout(5)
        .andThen(
            Commands.parallel(
                    arm.toSetpoint(armAngle),
                    Commands.waitUntil(arm::atSetpoint),
                    Commands.waitSeconds(0.1))
                .withTimeout(4))
        .andThen(
            orchestrator
                .turnToSpeaker()
                .alongWith(Commands.sequence(Commands.waitSeconds(1), orchestrator.shootBasic())));
  }
}
