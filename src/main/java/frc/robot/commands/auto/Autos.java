package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.utils.AllianceFlipUtil;
import frc.robot.FieldConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Autos {
  private final Drive drive;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Arm arm;
  private final Intake intake;

  private int FIRST_NOTE_POSITION = 0;
  private int SECOND_NOTE_POSITION = 0;
  private int THIRD_NOTE_POSITION = 0;

  Translation2d noteTranslation;
  Translation2d secondNoteTranslation;
  Translation2d thirdNoteTranslation;
  Translation2d speaker;

  public Autos(Drive drive, Shooter shooter, Indexer indexer, Arm arm, Intake intake) {
    this.drive = drive;
    this.shooter = shooter;
    this.indexer = indexer;
    this.arm = arm;
    this.intake = intake;
    this.speaker =
        AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d());
  }

  public enum StartingPosition {
    LEFT,
    CENTER,
    RIGHT
  }

  private void setNotePositions(StartingPosition position) {
    switch (position) {
      case LEFT -> {
        FIRST_NOTE_POSITION = 0;
        SECOND_NOTE_POSITION = 1;
        THIRD_NOTE_POSITION = 2;
      }
      case CENTER -> {
        FIRST_NOTE_POSITION = 1;
        SECOND_NOTE_POSITION = 2;
        THIRD_NOTE_POSITION = 0;
      }
      case RIGHT -> {
        FIRST_NOTE_POSITION = 2;
        SECOND_NOTE_POSITION = 1;
        THIRD_NOTE_POSITION = 0;
      }
    }

    this.noteTranslation =
        AllianceFlipUtil.apply(
            FieldConstants.StagingLocations.spikeTranslations[FIRST_NOTE_POSITION]);
    this.secondNoteTranslation =
        AllianceFlipUtil.apply(
            FieldConstants.StagingLocations.spikeTranslations[SECOND_NOTE_POSITION]);
    this.thirdNoteTranslation =
        AllianceFlipUtil.apply(
            FieldConstants.StagingLocations.spikeTranslations[THIRD_NOTE_POSITION]);

    Logger.recordOutput("Autos/setNotePositions/noteTranslation", noteTranslation);
    Logger.recordOutput("Autos/setNotePositions/secondNoteTranslation", secondNoteTranslation);
    Logger.recordOutput("Autos/setNotePositions/thirdNoteTranslation", thirdNoteTranslation);
  }

  private Command turnToSpeaker() {
    Supplier<Pose2d> targetPose =
        () ->
            new Pose2d(
                drive.getPose().getTranslation(),
                speaker.minus(drive.getPose().getTranslation()).getAngle());
    Logger.recordOutput("Autos/getSpeakerTargetPose/targetPose", targetPose.get());
    Logger.recordOutput("Autos/getSpeakerTargetPose/speaker", speaker);
    return Commands.defer(() -> new StraightDriveToPose(targetPose.get(), drive), Set.of(drive));
  }

  private Command driveToNote(Translation2d targetTranslation) {
    Supplier<Rotation2d> targetRotation =
        () -> targetTranslation.minus(drive.getPose().getTranslation()).getAngle();
    return Commands.defer(
        () -> new StraightDriveToPose(new Pose2d(targetTranslation, targetRotation.get()), drive),
        Set.of(drive));
  }

  private Command alignArm() {

    return Commands.defer(
        () ->
            arm.toSetpoint(
                new Rotation2d(
                    shooter.calculateShootingAngle(
                            drive.getPose().getTranslation().getDistance(speaker))
                        - ArmConstants.HORIZONTAL_OFFSET.getRadians())),
        Set.of(arm));
  }

  public Command oneNoteAuto() {
    return turnToSpeaker()
        .andThen(shooter.shoot(ShooterConstants.SHOOTER_READY_VELOCITY_RAD_PER_SEC));
  }

  public Command twoNoteAuto(StartingPosition position) {
    setNotePositions(position);
    return turnToSpeaker()
        .andThen(alignArm())
        .andThen(shooter.shoot(ShooterConstants.SHOOTER_READY_VELOCITY_RAD_PER_SEC))
        .andThen(
            Commands.parallel(
                driveToNote(noteTranslation),
                arm.toSetpoint(new Rotation2d().minus(ArmConstants.HORIZONTAL_OFFSET))))
        .andThen(Commands.parallel(intake.intake(), indexer.setIndexerPercentVelocity(0.25)))
        .andThen(turnToSpeaker())
        .andThen(alignArm())
        .andThen(shooter.shoot(ShooterConstants.SHOOTER_READY_VELOCITY_RAD_PER_SEC));
  }

  public Command threeNoteAuto(StartingPosition position) {
    setNotePositions(position);
    return turnToSpeaker()
        .andThen(alignArm())
        .andThen(
            shooter
                .shoot(ShooterConstants.SHOOTER_READY_VELOCITY_RAD_PER_SEC)
                .andThen(
                    Commands.parallel(
                        driveToNote(noteTranslation),
                        arm.toSetpoint(new Rotation2d().minus(ArmConstants.HORIZONTAL_OFFSET))))
                .andThen(
                    Commands.parallel(intake.intake(), indexer.setIndexerPercentVelocity(0.25)))
                .andThen(turnToSpeaker()))
        .andThen(alignArm())
        .andThen(shooter.shoot(ShooterConstants.SHOOTER_READY_VELOCITY_RAD_PER_SEC))
        .andThen(
            Commands.parallel(
                    driveToNote(secondNoteTranslation),
                    arm.toSetpoint(new Rotation2d().minus(ArmConstants.HORIZONTAL_OFFSET)))
                .andThen(
                    Commands.parallel(intake.intake(), indexer.setIndexerPercentVelocity(0.25))
                        .andThen(turnToSpeaker())
                        .andThen(alignArm())
                        .andThen(
                            shooter.shoot(ShooterConstants.SHOOTER_READY_VELOCITY_RAD_PER_SEC))));
  }

  public Command fourNoteAuto(StartingPosition position) {
    setNotePositions(position);
    return turnToSpeaker()
        .andThen(alignArm())
        .andThen(shooter.shoot(ShooterConstants.SHOOTER_READY_VELOCITY_RAD_PER_SEC))
        .andThen(
            Commands.parallel(
                driveToNote(noteTranslation),
                arm.toSetpoint(new Rotation2d().minus(ArmConstants.HORIZONTAL_OFFSET))))
        .andThen(Commands.parallel(intake.intake(), indexer.setIndexerPercentVelocity(0.25)))
        .andThen(turnToSpeaker())
        .andThen(alignArm())
        .andThen(shooter.shoot(ShooterConstants.SHOOTER_READY_VELOCITY_RAD_PER_SEC))
        .andThen(
            Commands.parallel(
                driveToNote(secondNoteTranslation),
                arm.toSetpoint(new Rotation2d().minus(ArmConstants.HORIZONTAL_OFFSET))))
        .andThen(
            Commands.parallel(intake.intake(), indexer.setIndexerPercentVelocity(0.25))
                .andThen(turnToSpeaker())
                .andThen(alignArm())
                .andThen(shooter.shoot(ShooterConstants.SHOOTER_READY_VELOCITY_RAD_PER_SEC)))
        .andThen(
            Commands.parallel(
                    driveToNote(thirdNoteTranslation),
                    arm.toSetpoint(new Rotation2d().minus(ArmConstants.HORIZONTAL_OFFSET)))
                .andThen(
                    Commands.parallel(intake.intake(), indexer.setIndexerPercentVelocity(0.25)))
                .andThen(turnToSpeaker())
                .andThen(alignArm())
                .andThen(shooter.shoot(ShooterConstants.SHOOTER_READY_VELOCITY_RAD_PER_SEC)));
  }
}
