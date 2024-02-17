package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.lib.utils.AllianceFlipUtil;
import frc.robot.FieldConstants;
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
  private final Shooter shooter;
  private final Indexer indexer;
  private final Arm arm;
  private final Intake intake;
  private final Pixy2 pixy2;
  private Translation2d noteTranslation;
  private Translation2d secondNoteTranslation;
  private Translation2d thirdNoteTranslation;

  private final Translation2d speaker =
      AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d());

  public Autos(Drive drive, Shooter shooter, Indexer indexer, Arm arm, Intake intake, Pixy2 pixy2) {
    this.drive = drive;
    this.shooter = shooter;
    this.indexer = indexer;
    this.arm = arm;
    this.intake = intake;
    this.pixy2 = pixy2;
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

  public Command turnToSpeaker() {
    Supplier<Pose2d> targetPose =
        () ->
            new Pose2d(
                drive.getPose().getTranslation(),
                speaker.minus(drive.getPose().getTranslation()).getAngle());
    Logger.recordOutput("Autos/Speaker", speaker);
    return Commands.defer(() -> new StraightDriveToPose(targetPose.get(), drive), Set.of(drive));
  }

  private Command driveToNote(Translation2d targetTranslation) {
    Supplier<Rotation2d> targetRotation =
        () -> targetTranslation.minus(drive.getPose().getTranslation()).getAngle();
    return Commands.defer(
        () -> new StraightDriveToPose(new Pose2d(targetTranslation, targetRotation.get()), drive),
        Set.of(drive));
  }

  public Command faceNote() {
    if (pixy2.seesNote()) {
      return Commands.defer(
          () ->
              new ProxyCommand(
                  () ->
                      new StraightDriveToPose(
                          new Pose2d(
                              drive.getPose().getTranslation(),
                              drive.getRotation().plus(Rotation2d.fromDegrees(pixy2.getAngle()))),
                          drive)),
          Set.of(drive));
    } else {
      return Commands.none();
    }
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
        .andThen(Commands.parallel(intake.intake(), indexer.setPercent(0.25)))
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
                    Commands.parallel(intake.intake(), indexer.setPercent(0.25)))
                .andThen(turnToSpeaker()))
        .andThen(alignArm())
        .andThen(shooter.shoot(ShooterConstants.SHOOTER_READY_VELOCITY_RAD_PER_SEC))
        .andThen(
            Commands.parallel(
                    driveToNote(secondNoteTranslation),
                    arm.toSetpoint(new Rotation2d().minus(ArmConstants.HORIZONTAL_OFFSET)))
                .andThen(
                    Commands.parallel(intake.intake(), indexer.setPercent(0.25))
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
        .andThen(Commands.parallel(intake.intake(), indexer.setPercent(0.25)))
        .andThen(turnToSpeaker())
        .andThen(alignArm())
        .andThen(shooter.shoot(ShooterConstants.SHOOTER_READY_VELOCITY_RAD_PER_SEC))
        .andThen(
            Commands.parallel(
                driveToNote(secondNoteTranslation),
                arm.toSetpoint(new Rotation2d().minus(ArmConstants.HORIZONTAL_OFFSET))))
        .andThen(
            Commands.parallel(intake.intake(), indexer.setPercent(0.25))
                .andThen(turnToSpeaker())
                .andThen(alignArm())
                .andThen(shooter.shoot(ShooterConstants.SHOOTER_READY_VELOCITY_RAD_PER_SEC)))
        .andThen(
            Commands.parallel(
                    driveToNote(thirdNoteTranslation),
                    arm.toSetpoint(new Rotation2d().minus(ArmConstants.HORIZONTAL_OFFSET)))
                .andThen(
                    Commands.parallel(intake.intake(), indexer.setPercent(0.25)))
                .andThen(turnToSpeaker())
                .andThen(alignArm())
                .andThen(shooter.shoot(ShooterConstants.SHOOTER_READY_VELOCITY_RAD_PER_SEC)));
  }
}
