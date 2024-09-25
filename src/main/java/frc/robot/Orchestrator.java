package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.utils.AllianceFlipUtil;
import frc.lib.utils.TunableNumber;
import frc.robot.commands.auto.StraightDriveToPose;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pixy2.Pixy2;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
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

  public Command deferredStraightDriveToPose(Supplier<Pose2d> pose) {
    return Commands.defer(() -> new StraightDriveToPose(pose.get(), drive), Set.of(drive));
  }

  /**
   * Drives to one of the notes pre positioned on the field.
   *
   * @param targetTranslation The Translation2d for one of the notes on the field during auto.
   * @return The command for driving to a note during auto.
   */
  public Command driveToNote(Supplier<Translation2d> targetTranslation) {
    Supplier<Rotation2d> targetRotation =
        () -> targetTranslation.get().minus(drive.getPose().getTranslation()).getAngle();
    return deferredStraightDriveToPose(
            () ->
                new Pose2d(
                    drive
                        .getPose()
                        .getTranslation()
                        .plus(
                            new Translation2d(
                                AllianceFlipUtil.applyRelative(Units.feetToMeters(0.531)), 0)),
                    targetRotation.get()))
        .withTimeout(2)
        .andThen(
            deferredStraightDriveToPose(
                    () -> new Pose2d(targetTranslation.get(), drive.getRotation()))
                .withTimeout(5));
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
    return deferredStraightDriveToPose(targetPose).withTimeout(3);
  }

  /**
   * Same as shootBasic, but for scoring in the amp instead, so we are using a slower speed.
   *
   * @return The cammand to shoot into the amp.
   */
  public Command shootAmp() {
    return Commands.sequence(
        shooter.manualShoot(ShooterConstants.AMP_SCORE_SPEED).withTimeout(0.5),
        Commands.parallel(
                indexer.setPercent(1), shooter.manualShoot(ShooterConstants.AMP_SCORE_SPEED))
            .until(() -> !indexer.getLimitSwitchPressed())
            .withTimeout(2));
  }

  /**
   * Sets the arm to amp scoring position
   *
   * @return The command to move the arm to setpoint
   */
  public Command alignArmAmp() {
    return Commands.parallel(
        arm.toSetpoint(ArmConstants.AMP_POSITION),
        Commands.waitUntil(arm::atSetpoint),
        Commands.waitSeconds(3));
  }

  public Command duck() {
    return arm.toSetpoint(ArmConstants.STOW.minus(Rotation2d.fromDegrees(5)))
        .alongWith(Commands.runOnce(() -> RobotState.getInstance().duck = true));
  }

  public Command unDuck() {
    return Commands.parallel(
            arm.toSetpoint(ArmConstants.AFTER_INTAKE_POS), Commands.waitUntil(arm::atSetpoint))
        .withTimeout(2)
        .beforeStarting(() -> RobotState.getInstance().duck = false);
  }

  public Command goToAmp() {
    TunableNumber AMP_DISTANCE_OFFSET = new TunableNumber("Orchestrator/AmpDistance", 1);
    Supplier<Pose2d> targetPose =
        () ->
            new Pose2d(
                AllianceFlipUtil.apply(FieldConstants.ampCenter.getX()),
                FieldConstants.ampCenter.getY() - AMP_DISTANCE_OFFSET.get(),
                new Rotation2d());
    return deferredStraightDriveToPose(targetPose)
        .andThen(
            deferredStraightDriveToPose(
                () ->
                    new Pose2d(
                        drive.getPose().getTranslation().getX(),
                        targetPose.get().getY() + AMP_DISTANCE_OFFSET.get() / 2,
                        AllianceFlipUtil.apply(FieldConstants.ampCenter)
                            .minus(drive.getPose().getTranslation())
                            .getAngle()
                            .minus(Rotation2d.fromRadians(Math.PI)))));
  }

  public Command spinUpFlywheel() {
    return (shooter
            .manualShoot(ShooterConstants.AMP_SCORE_SPEED)
            .onlyIf(() -> arm.getAngle() > Units.degreesToRadians(65)))
        .andThen(
            shooter
                .manualShoot(ShooterConstants.SHOOT_SPEED)
                .onlyIf(() -> arm.getAngle() < Units.degreesToRadians(65)));
  }

  /**
   * Shoots with 10.5 volts of power. Turns off the indexer and the shooter when the limit switch is
   * not pressed.
   *
   * @return The command to shoot the note.
   */
  public Command shootBasic() {
    return Commands.sequence(
            shooter
                .manualShoot(ShooterConstants.SHOOT_SPEED)
                .withTimeout(.5)
                .until(() -> shooter.atVelocity(ShooterConstants.SHOOT_SPEED))
                .andThen(
                    Commands.race(
                        Commands.waitSeconds(1.5),
                        shooter.manualShoot(ShooterConstants.SHOOT_SPEED))),
            Commands.parallel(
                    shooter.manualShoot(ShooterConstants.SHOOT_SPEED), indexer.setPercent(1))
                .withTimeout(1))
        .andThen(
            Commands.parallel(
                shooter.manualShoot(0).until(() -> true), indexer.setPercent(0).until(() -> true)));
  }

  public Command indexBasic() {
    return (indexer
            .setPercent(IndexerConstants.INDEX_SPEED.get())
            .until(() -> !indexer.getLimitSwitchPressed()))
        .andThen(
            Commands.race(
                    Commands.waitSeconds(0.5),
                    indexer.setPercent(IndexerConstants.INDEX_SPEED.get()))
                .andThen(
                    Commands.parallel(
                            arm.toSetpoint(ArmConstants.STOW), Commands.waitUntil(arm::atSetpoint))
                        .withTimeout(1)))
        .withTimeout(1);
  }

  public Command stopFlywheel() {
    return shooter.manualShoot(0).withTimeout(1);
  }

  /**
   * Calculates the angle to align the arm to the speaker from anywhere on the field. Does arcTan of
   * ((height of center of the speaker - height of the shooter) / distance to speaker)
   *
   * @return The command to move the arm to the correct setPoint for shooting from its current
   *     location.
   */
  public Command alignArmSpeaker(Supplier<Pose2d> robotPose) {
    return Commands.defer(
        () -> {
          Supplier<Double> distance =
              () ->
                  robotPose
                      .get()
                      .getTranslation()
                      .getDistance(
                          AllianceFlipUtil.apply(
                              FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d()));
          return arm.toSetpoint(
              () ->
                  Rotation2d.fromDegrees(
                      (-3.1419 * (distance.get() * distance.get()))
                          + (23.725 * distance.get())
                          - 30.103));
        },
        Set.of(arm));
    //    return Commands.defer(
    //        () ->
    //            arm.toSetpoint(
    //                new Rotation2d(
    //                    Math.abs(
    //                        Math.atan(
    //                            (FieldConstants.Speaker.centerSpeakerOpening.getZ()
    //                                    - Math.sin(arm.getAngle() +
    // ArmConstants.STOW.getRadians())
    //                                        * Units.inchesToMeters(28))
    //                                / drive.getPose().getTranslation().getDistance(speaker))))),
    //        Set.of(arm));
    //    return Commands.none();
  }

  /**
   * Aligns both the robot and then the arm to the speaker.
   *
   * @return The command to move the robot and the arm in preparation to shoot.
   */
  public Command fullAlignSpeaker(Supplier<Pose2d> robotPose) {
    return Commands.sequence(turnToSpeaker(), alignArmSpeaker(robotPose));
  }

  /**
   * Aligns the robot with the speaker and then aligns the arm and shoots. Uses the fullAlign and
   * shootBasic commands.
   *
   * @return The command to align both the robot and the arm, and then shoots at full power.
   */
  public Command fullAlignShootSpeaker(Supplier<Pose2d> robotPose) {
    return Commands.sequence(fullAlignSpeaker(robotPose), shootBasic());
  }

  /**
   * Turns on the intake along with the indexer until the limit switch is pressed.
   *
   * @return The command to intake the note and stopping afterwards.
   */
  public Command intakeBasic() {
    return Commands.sequence(
            arm.toSetpoint(ArmConstants.STOW.minus(Rotation2d.fromDegrees(5)))
                .until(arm::limitSwitchPressed)
                .withTimeout(2),
            Commands.parallel(
                    indexer.setPercent(IndexerConstants.INDEX_SPEED.get()), intake.intake())
                .until(() -> RobotState.getInstance().hasNote)
                .withTimeout(10))
        .andThen(pullBack());
  }

  public Command pullBack() {
    return Commands.parallel(
            indexer.setPercent(IndexerConstants.BACKUP_SPEED),
            shooter.manualShoot(-0.2),
            intake.stop())
        .withTimeout(IndexerConstants.BACKUP_TIME)
        .andThen(
            Commands.parallel(
                    arm.toSetpoint(ArmConstants.AFTER_INTAKE_POS).until(arm::atSetpoint),
                    indexer.setPercent(0).until(() -> true),
                    shooter.manualShoot(0).until(() -> true),
                    intake.stop().until(() -> true),
                    Commands.waitSeconds(0.5))
                .withTimeout(5));
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
    return Commands.parallel(shooter.manualShoot(-0.2), indexer.setPercent(-1.0));
  }

  /**
   * Expels a note from the whole robot.
   *
   * @return The command to release a note from all subsystems.
   */
  public Command expelFullRobot() {
    return Commands.parallel(expelIntake(), expelShindex());
  }

  public Command expelIntakeIndex() {
    return Commands.parallel(
        indexer.setPercent(-IndexerConstants.INDEX_SPEED.get()), expelIntake());
  }

  /**
   * Sets the arm to the home position, completely down.
   *
   * @return The command to move the arm to the home position.
   */
  public Command armToHome() {
    return arm.toSetpoint(ArmConstants.STOW);
  }
}
