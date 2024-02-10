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

  public Autos(Drive drive, Shooter shooter, Indexer indexer, Arm arm, Intake intake) {
    this.drive = drive;
    this.shooter = shooter;
    this.indexer = indexer;
    this.arm = arm;
    this.intake = intake;
  }

  public enum StartingPosition {
    LEFT,
    CENTER,
    RIGHT
  }

  private Supplier<Pose2d> getSpeakerTargetPose() {
    Translation2d speaker =
        AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d());
    Supplier<Pose2d> targetPose =
        () ->
            new Pose2d(
                drive.getPose().getTranslation(),
                speaker.minus(drive.getPose().getTranslation()).getAngle());
    Logger.recordOutput("Autos/getSpeakerTargetPose/targetPose", targetPose.get());
    Logger.recordOutput("Autos/getSpeakerTargetPose/speaker", speaker);
    return targetPose;
  }

  public Command oneNoteAuto() {
    return Commands.defer(
            () -> new StraightDriveToPose(getSpeakerTargetPose().get(), drive), Set.of(drive))
        .andThen(shooter.shoot(ShooterConstants.SHOOTER_READY_VELOCITY_RAD_PER_SEC));
  }

  public Command twoNoteAuto(StartingPosition position) {
    Pose2d notePose;
    switch (position) {
      case LEFT -> notePose = new Pose2d(); // TODO: Add the correct pose for the two note auto
      case CENTER -> notePose = new Pose2d();
      case RIGHT -> notePose = new Pose2d();
      default -> notePose = new Pose2d();
    }
    ;
    return Commands.defer(
            () -> new StraightDriveToPose(getSpeakerTargetPose().get(), drive), Set.of(drive))
        .andThen(
            shooter
                .shoot(ShooterConstants.SHOOTER_READY_VELOCITY_RAD_PER_SEC)
                .andThen(
                    Commands.parallel(
                        new StraightDriveToPose(notePose, drive),
                        arm.toSetpoint(
                            new Rotation2d(
                                -9000) // TODO: Add the correct setpoint for the two note auto
                            )))
                .andThen(
                    Commands.parallel(intake.intake(), indexer.setIndexerPercentVelocity(0.25)))
                .andThen(new StraightDriveToPose(getSpeakerTargetPose().get(), drive))
                .andThen(
                    arm.toSetpoint(
                        new Rotation2d(
                            shooter.calculateShootingAngle(
                                    drive
                                        .getPose()
                                        .getTranslation()
                                        .getDistance(
                                            AllianceFlipUtil.apply(
                                                FieldConstants.Speaker.centerSpeakerOpening
                                                    .toTranslation2d())))
                                - ArmConstants.HORIZONTAL_OFFSET.getRadians())))
                .andThen(shooter.shoot(ShooterConstants.SHOOTER_READY_VELOCITY_RAD_PER_SEC)));
  }
}
