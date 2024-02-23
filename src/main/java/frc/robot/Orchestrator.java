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
import org.littletonrobotics.junction.Logger;

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

  public Command turnToSpeaker() {
    Supplier<Pose2d> targetPose =
        () ->
            new Pose2d(
                drive.getPose().getTranslation(),
                speaker.minus(drive.getPose().getTranslation()).getAngle());
    Logger.recordOutput("Autos/Speaker", speaker);
    return Commands.defer(() -> new StraightDriveToPose(targetPose.get(), drive), Set.of(drive));
  }

  public Command driveToNote(Translation2d targetTranslation) {
    Supplier<Rotation2d> targetRotation =
        () -> targetTranslation.minus(drive.getPose().getTranslation()).getAngle();
    return Commands.defer(
        () -> new StraightDriveToPose(new Pose2d(targetTranslation, targetRotation.get()), drive),
        Set.of(drive));
  }

  public Command alignArm() {
    return Commands.defer(
        () ->
            arm.toSetpoint(
                new Rotation2d(
                    shooter.calculateShootingAngle(
                            drive.getPose().getTranslation().getDistance(speaker))
                        - ArmConstants.HORIZONTAL_OFFSET.getRadians())),
        Set.of(arm));
  }

  public Command shootBasic() {
    return Commands.sequence(
            Commands.parallel(
                shooter.shoot(9999)
                // Turn towards speaker.
                ),
            Commands.waitUntil(shooter::ShooterSpeedIsReady).withTimeout(2),
            indexer.setVolts(1),
            Commands.waitUntil(() -> !indexer.getLimitSwitchPressed()).withTimeout(2),
            Commands.parallel(indexer.setVolts(0), shooter.manualShoot(0)))
        .onlyIf(indexer::getLimitSwitchPressed);
  }

  public Command intakeBasic() {
    return Commands.sequence(
        indexer.setVolts(4),
        arm.toSetpoint(ArmConstants.HORIZONTAL_OFFSET), // TODO: Make setPoint for pickup position.
        Commands.waitUntil(arm::atSetpoint).withTimeout(2),
        Commands.parallel(intake.intake(), Commands.run(()->drive.runVelocity(new ChassisSpeeds(Units.inchesToMeters(10), 0.0,0.0)),drive)).until(() -> robotState.hasNote));
  }

  // Intakes after seeing note with Pixy2.
  public Command visionIntake() {
    return Commands.sequence(
        indexer.setVolts(4),
        arm.toSetpoint(ArmConstants.HORIZONTAL_OFFSET), // TODO: Make setpoint for pickup position.
        Commands.waitUntil(() -> arm.atSetpoint() && pixy2.seesNote()).withTimeout(2),
        Commands.parallel(intake.intake(), Commands.run(()->drive.runVelocity(new ChassisSpeeds(Units.inchesToMeters(10), 0.0,0.0)),drive)).until(() -> robotState.hasNote));
  }

  public Command expelFullRobot() {
    return Commands.parallel(
        shooter.manualShoot(-5), indexer.setVolts(-5.0), intake.release());
  }

  public Command expelShindex() {
    return Commands.parallel(shooter.manualShoot(-5.0), indexer.setVolts(-5.0));
  }

  public Command expelIntake() {
    return intake.release();
  }
}
