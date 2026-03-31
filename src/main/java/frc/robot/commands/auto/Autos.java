package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.utils.AllianceFlipUtil;
import frc.robot.Orchestrator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.rollers.IntakeRollers;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.Supplier;

public class Autos {
  private final Drive drive;
  private final Orchestrator orchestrator;
  private final Intake intake;
  private final IntakeRollers rollers;
  private final Shooter shooter;

  public Autos(
      Drive drive,
      Orchestrator orchestrator,
      Intake intake,
      IntakeRollers rollers,
      Shooter shooter) {
    this.drive = drive;
    this.orchestrator = orchestrator;
    this.intake = intake;
    this.rollers = rollers;
    this.shooter = shooter;
  }

  private static final Supplier<Pose2d> climb =
      () -> new Pose2d(1.583, 3.750, new Rotation2d(Units.degreesToRadians(180.000)));
  private static final Supplier<Pose2d> center = () -> new Pose2d(3.504, 4.019, new Rotation2d(0));
  private static final Supplier<Pose2d> CenterA =
      () -> new Pose2d(3.457, 4.941, new Rotation2d(Units.degreesToRadians(-55.305)));
  private static final Supplier<Pose2d> intakeSimplePoseA =
      () -> new Pose2d(7.7052903175354, 5.8276801109313965, Rotation2d.fromDegrees(0.0));
  private static final Supplier<Pose2d> overBumpNeutralPoseA =
      () -> new Pose2d(6.1, 5.574310302734375, Rotation2d.fromDegrees(0));

  private static final Supplier<Pose2d> intakeComplexFirstPoseA =
      () -> new Pose2d(6.862, 6.877, Rotation2d.fromDegrees(-90.0));
  // 6.862
  private static final Supplier<Pose2d> intakeComplexSecondPoseA =
      () -> new Pose2d(7.825, 6.877, Rotation2d.fromDegrees(-90));

  private static final Supplier<Pose2d> intakeComplexThirdPoseA =
      () -> new Pose2d(7.805, 4.461, Rotation2d.fromDegrees(-90));
  private static final Supplier<Pose2d> overBumpAlliancePoseA =
      () -> new Pose2d(3.0666706562042236, 5.574310302734375, Rotation2d.fromDegrees(0.0));
  private static final Supplier<Pose2d> overBumpAllianceAltPoseA =
      () -> new Pose2d(3.086160182952881, 5.437880039215088, Rotation2d.fromDegrees(0));
  private static final Supplier<Pose2d> shootFromCornerPoseA =
      () ->
          new Pose2d(
              3.086160182952881, 5.437880039215088, Rotation2d.fromRadians(-0.7553977556351216));
  private static final Supplier<Pose2d> intakeSimplePoseB =
      () -> new Pose2d(7.724780082702637, 2.436420202255249, Rotation2d.fromDegrees(0.0));
  private static final Supplier<Pose2d> overBumpAlliancePoseB =
      () -> new Pose2d(3.1251401901245117, 2.397440195083618, Rotation2d.fromDegrees(0.0));
  private static final Supplier<Pose2d> overBumpAllianceAltPoseB =
      () -> new Pose2d(3.1251401901245117, 2.397440195083618, Rotation2d.fromDegrees(0));
  private static final Supplier<Pose2d> shootFromCornerPoseB =
      () ->
          new Pose2d(
              3.1251401901245117, 2.397440195083618, Rotation2d.fromRadians(0.7358299996216245));
  private static final Supplier<Pose2d> depotPoseStopPoint =
      () -> new Pose2d(1.7998201847076416, 5.983600616455078, Rotation2d.fromDegrees(180));
  private static final Supplier<Pose2d> depotPose =
      () -> new Pose2d(0.45501017570495605, 5.983600616455078, Rotation2d.fromDegrees(180));

  // pp

  private static final Supplier<Pose2d> startAside =
      () -> AllianceFlipUtil.apply(new Pose2d(3.645, 5.520, new Rotation2d(0.000)));
  private static final Supplier<Pose2d> startBside =
      () -> AllianceFlipUtil.apply(new Pose2d(3.645, 2.516, new Rotation2d(0.000)));

  public Command CenterA() {
    return Commands.sequence(
        Commands.runOnce(() -> drive.setPose(AllianceFlipUtil.apply(CenterA.get()))),
        drive.getAutonomousCommand("CA-out-intake"),
        new StraightDriveToPose(climb.get(), drive).withTimeout(2.0));
  }

  public Command testPath() {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                drive.setPose(
                    AllianceFlipUtil.apply(
                        new Pose2d(3.213, 5.600, new Rotation2d(Units.degreesToRadians(0)))))),
        drive.getAutonomousCommand("test path"));
  }

  public Command Bummmmpar() {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                drive.setPose(
                    AllianceFlipUtil.apply(
                        new Pose2d(3.213, 5.600, new Rotation2d(Units.degreesToRadians(0)))))),
        new DriveToPose(
            drive,
            () -> AllianceFlipUtil.apply(new Pose2d(2.798, 5.440, Rotation2d.fromDegrees(0)))),
        new DriveToPose(
            drive,
            () -> AllianceFlipUtil.apply(new Pose2d(6.714, 5.440, Rotation2d.fromDegrees(0)))),
        new DriveToPose(
            drive,
            () -> AllianceFlipUtil.apply(new Pose2d(2.798, 5.440, Rotation2d.fromDegrees(0)))));
  }

  public Command ppACycleLeft() {
    return Commands.sequence(
        Commands.runOnce(() -> drive.setPose(startAside.get())),
        Commands.deadline(
                drive.getAutonomousCommand("A-Cycle-LeftSweep"),
                intake.extendToAngleAndIntake(IntakeConstants.EXTEND_POS).withTimeout(5.5),
                orchestrator.spinUpShooterHub())
            .withTimeout(14.5),
        orchestrator.aimToHub().withTimeout(2.5),
        Commands.parallel(orchestrator.spinUpShooter(1215), orchestrator.feedUp()).withTimeout(2.5),
        Commands.parallel(
            intake.extendToAngle(IntakeConstants.COLLAPSE_POS),
            orchestrator.spinUpShooter(1214),
            orchestrator.feedUp()));
  }

  public Command ppBCycleRight() {
    return Commands.sequence(
        Commands.runOnce(() -> drive.setPose(startAside.get())),
        Commands.deadline(
                drive.getAutonomousCommand("B-Cycle-RightSweep"),
                intake.extendToAngleAndIntake(IntakeConstants.EXTEND_POS).withTimeout(5.9),
                orchestrator.spinUpShooterHub())
            .withTimeout(14.0),
        orchestrator.aimToHub().withTimeout(2.5),
        Commands.parallel(orchestrator.spinUpShooter(1215), orchestrator.feedUp()).withTimeout(2.5),
        Commands.parallel(
            intake.extendToAngleAndIntake(IntakeConstants.COLLAPSE_POS),
            orchestrator.spinUpShooter(1215),
            orchestrator.feedUp()));
  }

  public Command EightBalls() {
    return Commands.sequence(
        Commands.deadline(
            orchestrator.driveToHub().withTimeout(3.0), orchestrator.spinUpShooterHub()),
        Commands.parallel(orchestrator.spinUpShooterHub(), orchestrator.feedUp()));
  }

  public Command ACCManuelAuto() {
    return Commands.sequence(
        Commands.deadline(
            new DriveToPose(drive, () -> AllianceFlipUtil.apply(intakeSimplePoseA.get()))
                .withTimeout(5),
            Commands.parallel(intake.extendToAngleAndIntake(IntakeConstants.EXTEND_POS))),
        new DriveToPose(drive, () -> AllianceFlipUtil.apply(overBumpAlliancePoseA.get()))
            .withTimeout(5),
        rollers.stopIntakeCommand().withTimeout(0.05),
        Commands.deadline(
            orchestrator.driveToHub().withTimeout(3), orchestrator.spinUpShooterHub()),
        Commands.parallel(
            orchestrator.spinUpShooterHub(),
            orchestrator.feedUp(),
            Commands.waitSeconds(2).andThen(intake.extendToAngleAndIntake(0.0))));
  }

  public Command ACCManuelImprovedComplexIntake() {
    return Commands.sequence(
        Commands.deadline(
            Commands.sequence(
                new DriveToPose(drive, () -> AllianceFlipUtil.apply(overBumpNeutralPoseA.get()))
                    .withTimeout(2),
                new DriveToPose(drive, () -> AllianceFlipUtil.apply(intakeComplexFirstPoseA.get())),
                new DriveToPose(drive, () -> AllianceFlipUtil.apply(intakeComplexSecondPoseA.get()))
                    .withTimeout(1.9),
                new DriveToPose(drive, () -> AllianceFlipUtil.apply(intakeComplexThirdPoseA.get()))
                    .withTimeout(3),
                new DriveToPose(drive, () -> AllianceFlipUtil.apply(overBumpNeutralPoseA.get()))),
            Commands.parallel(intake.extendToAngleAndIntake(IntakeConstants.EXTEND_POS))),
        new DriveToPose(drive, () -> AllianceFlipUtil.apply(overBumpAllianceAltPoseA.get()))
            .withTimeout(3.5),
        rollers.stopIntakeCommand().withTimeout(0.05),
        Commands.deadline(
            new DriveToPose(drive, () -> AllianceFlipUtil.apply(shootFromCornerPoseA.get()))
                .withTimeout(4),
            orchestrator.spinUpShooter(1240)),
        Commands.parallel(
            orchestrator.spinUpShooter(1240),
            orchestrator.feedUp(),
            Commands.waitSeconds(2).andThen(intake.extendToAngleAndIntake(0.0))));
  }

  public Command ADepot() {
    return Commands.sequence(
        new DriveToPose(drive, () -> AllianceFlipUtil.apply(depotPoseStopPoint.get()))
            .withTimeout(2),
        Commands.deadline(
            new DriveToPose(drive, () -> AllianceFlipUtil.apply(depotPose.get())).withTimeout(3.5),
            Commands.parallel(intake.extendToAngleAndIntake(IntakeConstants.EXTEND_POS))),
        rollers.stopIntakeCommand().withTimeout(0.05),
        Commands.deadline(
            new DriveToPose(drive, () -> AllianceFlipUtil.apply(shootFromCornerPoseA.get()))
                .withTimeout(2),
            orchestrator.spinUpShooter(1250)),
        Commands.deadline(
            Commands.waitSeconds(5),
            orchestrator.spinUpShooter(1250),
            orchestrator.feedUp(),
            Commands.waitSeconds(2).andThen(intake.extendToAngleAndIntake(0.0))),
        rollers.stopIntakeCommand().withTimeout(0.05),
        shooter.stop(),
        new DriveToPose(drive, () -> AllianceFlipUtil.apply(overBumpAllianceAltPoseA.get()))
            .withTimeout(2),
        Commands.deadline(
            new DriveToPose(drive, () -> AllianceFlipUtil.apply(intakeSimplePoseA.get()))
                .withTimeout(3.5),
            Commands.parallel(
                intake.extendToAngleAndIntake(IntakeConstants.EXTEND_POS),
                orchestrator.preloadBalls())));
  }

  public Command ACCManuelAutoAlt() {
    return Commands.sequence(
        Commands.deadline(
            new DriveToPose(drive, () -> AllianceFlipUtil.apply(intakeSimplePoseA.get()))
                .withTimeout(5),
            intake.extendToAngleAndIntake(IntakeConstants.EXTEND_POS)),
        new DriveToPose(drive, () -> AllianceFlipUtil.apply(overBumpAllianceAltPoseA.get()))
            .withTimeout(3.5),
        rollers.stopIntakeCommand().withTimeout(0.05),
        Commands.deadline(
            new DriveToPose(drive, () -> AllianceFlipUtil.apply(shootFromCornerPoseA.get()))
                .withTimeout(2),
            orchestrator.spinUpShooter(1240)),
        Commands.parallel(
            orchestrator.spinUpShooter(1240),
            orchestrator.feedUp(),
            Commands.waitSeconds(2).andThen(intake.extendToAngleAndIntake(0.0))));
  }

  public Command ACCManuelAutoOverBump() {
    return Commands.sequence(
        Commands.deadline(
            new DriveToPose(drive, () -> AllianceFlipUtil.apply(intakeSimplePoseA.get()))
                .withTimeout(5),
            Commands.parallel(
                intake.holdAngleAndIntake(IntakeConstants.EXTEND_POS),
                orchestrator.preloadBalls())),
        new DriveToPose(drive, () -> AllianceFlipUtil.apply(overBumpAllianceAltPoseA.get()))
            .withTimeout(3.5),
        rollers.stopIntakeCommand().withTimeout(0.05),
        Commands.deadline(
            new DriveToPose(drive, () -> AllianceFlipUtil.apply(shootFromCornerPoseA.get()))
                .withTimeout(2),
            orchestrator.spinUpShooter(1250)),
        Commands.deadline(
            Commands.waitSeconds(5),
            orchestrator.spinUpShooter(1250),
            orchestrator.feedUp(),
            Commands.waitSeconds(2).andThen(intake.extendToAngleAndIntake(0.0))),
        rollers.stopIntakeCommand().withTimeout(0.05),
        shooter.stop(),
        new DriveToPose(drive, () -> AllianceFlipUtil.apply(overBumpAllianceAltPoseA.get()))
            .withTimeout(2),
        Commands.deadline(
            new DriveToPose(drive, () -> AllianceFlipUtil.apply(intakeSimplePoseA.get()))
                .withTimeout(3.5),
            Commands.parallel(
                intake.holdAngleAndIntake(IntakeConstants.EXTEND_POS),
                orchestrator.preloadBalls())));
  }

  public Command BCCManuelAuto() {
    return Commands.sequence(
        Commands.deadline(
            new DriveToPose(drive, () -> AllianceFlipUtil.apply(intakeSimplePoseB.get()))
                .withTimeout(5),
            Commands.parallel(
                intake.holdAngleAndIntake(IntakeConstants.EXTEND_POS),
                orchestrator.preloadBalls())),
        new DriveToPose(drive, () -> AllianceFlipUtil.apply(overBumpAllianceAltPoseB.get()))
            .withTimeout(5),
        rollers.stopIntakeCommand().withTimeout(0.05),
        Commands.deadline(
            orchestrator.driveToHub().withTimeout(3), orchestrator.spinUpShooterHub()),
        Commands.parallel(
            orchestrator.spinUpShooterHub(),
            orchestrator.feedUp(),
            Commands.waitSeconds(2).andThen(intake.extendToAngleAndIntake(0.0))));
  }

  public Command BCCManuelAutoAlt() {
    return Commands.sequence(
        Commands.deadline(
            new DriveToPose(drive, () -> AllianceFlipUtil.apply(intakeSimplePoseB.get()))
                .withTimeout(5),
            Commands.parallel(
                intake.holdAngleAndIntake(IntakeConstants.EXTEND_POS),
                orchestrator.preloadBalls())),
        new DriveToPose(drive, () -> AllianceFlipUtil.apply(overBumpAllianceAltPoseB.get()))
            .withTimeout(3.5),
        rollers.stopIntakeCommand().withTimeout(0.05),
        Commands.deadline(
            new DriveToPose(drive, () -> AllianceFlipUtil.apply(shootFromCornerPoseB.get()))
                .withTimeout(2),
            orchestrator.spinUpShooter(1240)),
        Commands.parallel(
            orchestrator.spinUpShooter(1240),
            orchestrator.feedUp(),
            Commands.waitSeconds(2).andThen(intake.extendToAngleAndIntake(0.0))));
  }

  public Command BCCManuelAutoOverBump() {
    return Commands.sequence(
        Commands.deadline(
            new DriveToPose(drive, () -> AllianceFlipUtil.apply(intakeSimplePoseB.get()))
                .withTimeout(5),
            Commands.parallel(
                intake.holdAngleAndIntake(IntakeConstants.EXTEND_POS),
                orchestrator.preloadBalls())),
        new DriveToPose(drive, () -> AllianceFlipUtil.apply(overBumpAllianceAltPoseB.get()))
            .withTimeout(3.5),
        rollers.stopIntakeCommand().withTimeout(0.05),
        Commands.deadline(
            new DriveToPose(drive, () -> AllianceFlipUtil.apply(shootFromCornerPoseB.get()))
                .withTimeout(2),
            orchestrator.spinUpShooter(1250)),
        Commands.deadline(
            Commands.waitSeconds(5),
            orchestrator.spinUpShooter(1250),
            orchestrator.feedUp(),
            Commands.waitSeconds(2).andThen(intake.extendToAngleAndIntake(0.0))),
        rollers.stopIntakeCommand().withTimeout(0.05),
        shooter.stop(),
        new DriveToPose(drive, () -> AllianceFlipUtil.apply(overBumpAllianceAltPoseB.get()))
            .withTimeout(2),
        Commands.deadline(
            new DriveToPose(drive, () -> AllianceFlipUtil.apply(intakeSimplePoseB.get()))
                .withTimeout(3.5),
            Commands.parallel(
                intake.holdAngleAndIntake(IntakeConstants.EXTEND_POS),
                orchestrator.preloadBalls())));
  }
}
