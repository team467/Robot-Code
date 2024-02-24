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
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pixy2.Pixy2;
import frc.robot.subsystems.robotstate.RobotState;
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
  private final RobotState robotState;
  private final Translation2d speaker =
      AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d());

  public Orchestrator(
      Drive drive,
      Intake intake,
      Indexer indexer,
      Shooter shooter,
      Pixy2 pixy2,
      Arm arm,
      RobotState robotState) {
    this.drive = drive;
    this.intake = intake;
    this.indexer = indexer;
    this.shooter = shooter;
    this.pixy2 = pixy2;
    this.arm = arm;
    this.robotState = robotState;
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
                                    - Math.sin(arm.getAngle() - Units.degreesToRadians(14.95))
                                        * Units.inchesToMeters(28))
                                / drive.getPose().getTranslation().getDistance(speaker))))),
        Set.of(arm));
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
                speaker.minus(drive.getPose().getTranslation()).getAngle());
    return Commands.defer(() -> new StraightDriveToPose(targetPose.get(), drive), Set.of(drive));
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
                FieldConstants.ampCenter.getX(),
                FieldConstants.ampCenter.getY() - 1,
                new Rotation2d());
    return Commands.defer(() -> new StraightDriveToPose(targetPose.get(), drive), Set.of(drive))
        .andThen(
            Commands.defer(
                () ->
                    new StraightDriveToPose(
                        new Pose2d(
                            targetPose.get().getX(),
                            targetPose.get().getY() + 0.5,
                            drive
                                .getPose()
                                .getTranslation()
                                .plus(FieldConstants.ampCenter)
                                .getAngle()),
                        drive),
                Set.of(drive)));
  }

  public Command scoreAmp() {
    return Commands.parallel(goToAmp(), alignArmAmp()).andThen(shootBasic());
  }

  /**
   * Shoots with a velocity setpoint of 9999 rad per seconds, this needs to be tuned. Turns off the
   * indexer and the shooter when the limit switch is not pressed.
   *
   * @return The command to shoot the note.
   */
  public Command shootBasic() {
    return Commands.sequence(
            shooter.manualShoot(12),
            Commands.waitUntil(shooter::ShooterSpeedIsReady).withTimeout(2),
            indexer.setVolts(1),
            Commands.waitUntil(() -> !indexer.getLimitSwitchPressed()).withTimeout(2),
            Commands.parallel(indexer.setVolts(0), shooter.manualShoot(0)))
        .onlyIf(indexer::getLimitSwitchPressed);
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
   * @return The command to align both the robot and the arm, and then shoot.
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
        indexer.setVolts(4),
        arm.toSetpoint(ArmConstants.HORIZONTAL_OFFSET), // TODO: Make setPoint for pickup position.
        Commands.waitUntil(arm::atSetpoint).withTimeout(2),
        intake.intake().until(() -> robotState.hasNote));
  }

  /**
   * Turns on the intake while driving for 2 seconds, both happening in parallel.
   *
   * @return The command to move the robot and intake.
   */
  public Command driveWhileIntaking() {
    return Commands.parallel(
        intake.intake().until(() -> robotState.hasNote),
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
        arm.toSetpoint(ArmConstants.HORIZONTAL_OFFSET), // TODO: Make setpoint for pickup position.
        Commands.waitUntil(() -> arm.atSetpoint() && pixy2.seesNote()).withTimeout(2),
        intake.intake().until(() -> robotState.hasNote));
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
    return Commands.parallel(shooter.manualShoot(-5.0), indexer.setVolts(-12.0));
  }

  /**
   * Expels a note from the whole robot.
   *
   * @return The command to release a note from all subsystems.
   */
  public Command expelFullRobot() {
    return Commands.parallel(expelIntake(), expelShindex());
  }
}
