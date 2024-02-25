package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.utils.AllianceFlipUtil;
import frc.robot.commands.auto.StraightDriveToPose;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pixy2.Pixy2;
import frc.robot.subsystems.shooter.Shooter;
import java.util.Set;
import java.util.function.Supplier;

public class Orchestrator {
  private final Drive drive;
  private final Intake intake;
  private final Indexer indexer;
  private final Shooter shooter;
  private final Pixy2 pixy2;
  private final Arm arm;
  private final Translation2d speaker =
      AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d());

  public Orchestrator(
      Drive drive, Intake intake, Indexer indexer, Shooter shooter, Pixy2 pixy2, Arm arm) {
    this.drive = drive;
    this.intake = intake;
    this.indexer = indexer;
    this.shooter = shooter;
    this.pixy2 = pixy2;
    this.arm = arm;
  }

  /**
   * Drives to one of the notes pre positioned on the field.
   *
   * @param targetTranslation The Translation2d for one of the notes on the field during auto.
   * @return The command for driving to a note during auto.
   */
  public Command driveToNote(Translation2d targetTranslation) {
    Supplier<Rotation2d> targetRotation =
        () -> targetTranslation.minus(drive.getPose().getTranslation()).getAngle();
    return Commands.defer(
        () -> new StraightDriveToPose(new Pose2d(targetTranslation, targetRotation.get()), drive),
        Set.of(drive));
  }

  /**
   * Calculates the angle to turn to the speaker, and turns.
   *
   * @return The command for the robot to turn.
   */
  public Command turnToSpeaker() {
    Supplier<Pose2d> targetPose =
        () ->
            new Pose2d(
                drive.getPose().getTranslation(),
                speaker
                    .minus(drive.getPose().getTranslation())
                    .getAngle()
                    .minus(Rotation2d.fromDegrees(180)));
    return Commands.defer(() -> new StraightDriveToPose(targetPose.get(), drive), Set.of(drive));
  }

  /**
   * Same as shootBasic, but for scoring in the amp instead, so we are using a slower speed.
   *
   * @return The cammand to shoot into the amp.
   */
  public Command shootAmp() {
    return Commands.sequence(
            shooter.manualShoot(3.5).withTimeout(0.5),
            Commands.parallel(indexer.setPercent(1), shooter.manualShoot(3.5))
                    .until(() -> !indexer.getLimitSwitchPressed()).withTimeout(2));
  }

  /**
   * Sets the arm to amp scoring position
   *
   * @return The command to move the arm to setpoint
   */
  public Command alignArmAmp() {
    return arm.toSetpoint(ArmConstants.AMP_POSITION);
  }

  public Command goToAmp() {
    Supplier<Pose2d> targetPose =
        () ->
            new Pose2d(
                AllianceFlipUtil.apply(FieldConstants.ampCenter.getX()),
                FieldConstants.ampCenter.getY() - 1,
                new Rotation2d());
    return Commands.defer(() -> new StraightDriveToPose(targetPose.get(), drive), Set.of(drive))
        .andThen(
            Commands.defer(
                () ->
                    new StraightDriveToPose(
                        new Pose2d(
                            drive.getPose().getTranslation().getX(),
                            targetPose.get().getY() + 0.5,
                            AllianceFlipUtil.apply(FieldConstants.ampCenter)
                                .minus(drive.getPose().getTranslation())
                                .getAngle()
                                .minus(Rotation2d.fromRadians(Math.PI))),
                        drive),
                Set.of(drive)));
  }

  /**
   * Uses goToAmp(), alignArmAmp(), and shootBasic() to move the robot to the amp and then line up
   * and shoot.
   *
   * @return The command for scoring in the amp from any spot on the field.
   */
  public Command scoreAmp() {
    return Commands.parallel(goToAmp(), alignArmAmp()).andThen(shootAmp());
  }

  /**
   * Shoots with a velocity setpoint of 9999 rad per seconds, this needs to be tuned. Turns off the
   * indexer and the shooter when the limit switch is not pressed.
   *
   * @return The command to shoot the note.
   */
  public Command shootBasic() {
    return Commands.sequence(
        shooter.manualShoot(10).withTimeout(2),
        Commands.parallel(shooter.manualShoot(10), indexer.setPercent(1))
            .until(() -> !indexer.getLimitSwitchPressed())
            .withTimeout(5));
  }

  /**
   * Calculates the angle to align the arm to the speaker from anywhere on the field. Does arcTan of
   * ((height of center of the speaker - height of the shooter) / distance to speaker)
   *
   * @return The command to move the arm to the correct setPoint for shooting from its current
   *     location.
   */
  public Command alignArmSpeaker() {
    return Commands.defer(
        () ->
            arm.toSetpoint(
                new Rotation2d(
                    Math.abs(
                        Math.atan(
                            (FieldConstants.Speaker.centerSpeakerOpening.getZ()
                                    - Math.sin(arm.getAngle() - Units.degreesToRadians(13.95))
                                        * Units.inchesToMeters(28))
                                / drive.getPose().getTranslation().getDistance(speaker))))),
        Set.of(arm));
  }

  /**
   * Aligns both the robot and then the arm to the speaker.
   *
   * @return The command to move the robot and the arm in preparation to shoot.
   */
  public Command fullAlignSpeaker() {
    return Commands.sequence(turnToSpeaker(), alignArmSpeaker());
  }

  /**
   * Aligns the robot with the speaker and then aligns the arm and shoots. Uses the fullAlign and
   * shootBasic commands.
   *
   * @return The command to align both the robot and the arm, and then shoots at full power.
   */
  public Command fullAlignShootSpeaker() {
    return Commands.sequence(fullAlignSpeaker(), shootBasic());
  }

  /**
   * Turns on the intake along with the indexer until the limit switch is pressed.
   *
   * @return The command to intake the note and stopping afterwards.
   */
  public Command intakeBasic() {
    return Commands.sequence(
            arm.toSetpoint(ArmConstants.OFFSET).until(arm::atSetpoint).withTimeout(2),
            Commands.parallel(
                    indexer.setPercent(IndexerConstants.INDEX_SPEED.get()),
                    intake.intake())
                .until(indexer::getLimitSwitchPressed).withTimeout(1))
        .finallyDo(() -> indexer.setPercent(-0.6).withTimeout(2));
  }

  /**
   * Turns on the intake while driving for 2 seconds, both happening in parallel.
   *
   * @return The command to move the robot and intake.
   */
  public Command driveWhileIntaking() {
    return Commands.parallel(
        intake.intake().until(indexer::getLimitSwitchPressed),
        Commands.run(
                () -> drive.runVelocity(new ChassisSpeeds(Units.inchesToMeters(10), 0.0, 0.0)),
                drive)
            .withTimeout(2));
  }

  /**
   * Turn on the intake when we see a note.
   *
   * @return The command to intake after a note is seen.
   */
  // Intakes after seeing note with Pixy2.
  public Command basicVisionIntake() {
    return Commands.sequence(
        indexer.setVolts(4),
        arm.toSetpoint(ArmConstants.OFFSET),
        Commands.waitUntil(() -> arm.atSetpoint() && pixy2.seesNote()).withTimeout(2),
        intake.intake().until(indexer::getLimitSwitchPressed));
  }

  /* TODO: Complete once pixy is done. Will drive towards note using the angle and distance supplied by the pixy2.
  Will use intakeBasic in parallel. */
  public Command fullVisionIntake() {
    return null;
  }

  /**
   * Expels the intake if we want to get rid of a note.
   *
   * @return The command to release a note in the intake.
   */
  public Command expelIntake() {
    return intake.release();
  }

  /**
   * Expels the intake if we want to get rid of a note in shooter and indexer.
   *
   * @return The command to release a note in the shooter and indexer.
   */
  public Command expelShindex() {
    return Commands.parallel(shooter.manualShoot(-0.2 * 12), indexer.setPercent(-1.0));
  }

  /**
   * Expels a note from the whole robot.
   *
   * @return The command to release a note from all subsystems.
   */
  public Command expelFullRobot() {
    return Commands.parallel(expelIntake(), expelShindex());
  }

  /**
   * Uses intakeBasic and shootBasic in order to shoot a note while lined up with the speaker,
   * doesn't move the arm.
   *
   * @return The command to intake a note and then shoot that note, more for testing purposes.
   */
  public Command intakeAndShootSpeaker() {
    return Commands.sequence(intakeBasic(), shootBasic());
  }

  /**
   * Sets the arm to the home position, completely down.
   *
   * @return The command to move the arm to the home position.
   */
  public Command armToHome() {
    return arm.toSetpoint(ArmConstants.OFFSET);
  }
}
