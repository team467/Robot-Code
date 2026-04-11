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
  public record AutoPositions(
      Pose2d intakeSimplePose, Pose2d overBumpAllianceAlt, Pose2d shootFromCorner) {}

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

  /**
   * Helper function for a single cycle auto with a constant shooting speed
   *
   * @param path The pathplanner path file name to use
   * @param startPose A supplier that returns that starting position of the robot for this path
   * @return A command that follows the path and shoots
   */
  private Command ppCycle(String path, Supplier<Pose2d> startPose) {
    return Commands.sequence(
        Commands.runOnce(() -> drive.setPose(startPose.get())),
        Commands.deadline(
                drive.getAutonomousCommand(path),
                intake.extendToAngleAndIntake(IntakeConstants.EXTEND_POS).withTimeout(5.5),
                orchestrator.spinUpShooterHub())
            .withTimeout(14.5),
        orchestrator.aimToHub().withTimeout(2.5),
        Commands.parallel(orchestrator.spinUpShooter(1215), orchestrator.feedUp()).withTimeout(2.5),
        Commands.parallel(
            intake.extendToAngleAndIntake(IntakeConstants.COLLAPSE_POS),
            orchestrator.spinUpShooter(1214),
            orchestrator.feedUp()));
  }

  /**
   * Helper function for a single cycle auto that uses shooter regression to shoot into the hub
   *
   * @param path The pathplanner path file name to use
   * @param startPose A Supplier that returns the starting position of the robot for this path
   * @return A command that follows the path and shoots
   */
  private Command ppCycleRegression(String path, Supplier<Pose2d> startPose) {
    return Commands.sequence(
        Commands.runOnce(() -> drive.setPose(startPose.get())),
        Commands.deadline(
                drive.getAutonomousCommand(path),
                intake.extendToAngleAndIntake(IntakeConstants.EXTEND_POS).withTimeout(5.5))
            .withTimeout(14.5),
        Commands.deadline(
            orchestrator.aimToHub().withTimeout(2.5),
            orchestrator.spinUpShooterDistance(orchestrator.getHubDistance())),
        Commands.parallel(
                orchestrator.spinUpShooterDistance(orchestrator.getHubDistance()),
                orchestrator.feedUp())
            .withTimeout(2.5),
        Commands.parallel(
            intake.extendToAngleAndIntake(IntakeConstants.COLLAPSE_POS),
            orchestrator.spinUpShooterDistance(orchestrator.getHubDistance()),
            orchestrator.feedUp()));
  }

  /**
   * Helper function for a double cycle auto that uses shooter regression to shoot into the hub
   * twice (once after each cycle)
   *
   * @param path1 The pathplanner path file name to use for the first cycle
   * @param path2 The pathplanner path file name to use for the second cycle
   * @param startPose The starting position of the robot for the first path and the position it ends
   *     up in after the path
   * @return A command that follows the paths and shoots
   */
  private Command pp2CycleRegression(String path1, String path2, Supplier<Pose2d> startPose) {
    return ppCycleRegression(path1, startPose)
        .withTimeout(2.5)
        .andThen(ppCycleRegression(path2, startPose));
  }

  // Pathplanner autos using the helper functions

  /**
   * Left side pathplanner auto for a single cycle with fixed shooting distance
   *
   * @return A command that executes the auto
   */
  public Command ppACycleLeft() {
    return ppCycle("A-Cycle-LeftSweep", startAside);
  }

  /**
   * Right side pathplanner auto for a single cycle with fixed shooting distance
   *
   * @return A command that executes the auto
   */
  public Command ppBCycleRight() {
    return ppCycle("B-Cycle-RightSweep", startBside);
  }

  /**
   * Left side pathplanner auto for a single cycle that supports any shooting distance
   *
   * @return A command that executes the auto
   */
  public Command ppACycleLeftRegression() {
    return ppCycleRegression("A-Cycle-LeftSweep", startAside);
  }

  /**
   * Right side pathplanner auto for a single cycle that supports any shooting distance
   *
   * @return A command that executes the auto
   */
  public Command ppBCycleRightRegression() {
    return ppCycleRegression("B-Cycle-RightSweep", startBside);
  }

  /**
   * Left side pathplanner auto for 2 cycles that support any shooting distance
   *
   * @return A comand that executes the auto
   */
  public Command ppA2CycleRightRegression() {
    return pp2CycleRegression("A-Cycle1", "A-Cycle2", startBside);
  }

  /**
   * Right side pathplanner auto for 2 cycles that support any shooting distance
   *
   * @return A comand that executes the auto
   */
  public Command ppB2CycleRightRegression() {
    return pp2CycleRegression("B-Cycle1", "B-Cycle2", startBside);
  }

  /**
   * Gets relevant poses for the autos
   *
   * @param isA Whether to get the poses for A path or B path (true for A, false for B)
   * @return The relevant poses for the calling auto
   */
  AutoPositions getPoses(boolean isA) {
    if (isA) {
      return new AutoPositions(
          intakeSimplePoseA.get(), overBumpAllianceAltPoseA.get(), shootFromCornerPoseA.get());
    }
    return new AutoPositions(
        intakeSimplePoseB.get(), overBumpAllianceAltPoseB.get(), shootFromCornerPoseB.get());
  }

  private Command ccManualAuto(boolean isA) {
    AutoPositions poses = getPoses(isA);

    return Commands.sequence(
        Commands.deadline(
            new DriveToPose(drive, () -> AllianceFlipUtil.apply(poses.intakeSimplePose))
                .withTimeout(5),
            intake.extendToAngleAndIntake(IntakeConstants.EXTEND_POS)),
        new DriveToPose(drive, () -> AllianceFlipUtil.apply(poses.overBumpAllianceAlt))
            .withTimeout(5),
        rollers.stopIntakeCommand().withTimeout(0.05),
        Commands.deadline(
            orchestrator.driveToHub().withTimeout(3), orchestrator.spinUpShooterHub()),
        Commands.parallel(
            orchestrator.spinUpShooterHub(),
            orchestrator.feedUp(),
            Commands.waitSeconds(2).andThen(intake.extendToAngleAndIntake(0.0))));
  }

  private Command ccManualAutoAlt(boolean isA) {
    AutoPositions poses = getPoses(isA);

    return Commands.sequence(
        Commands.deadline(
            new DriveToPose(drive, () -> AllianceFlipUtil.apply(poses.intakeSimplePose))
                .withTimeout(5),
            intake.extendToAngleAndIntake(IntakeConstants.EXTEND_POS)),
        new DriveToPose(drive, () -> AllianceFlipUtil.apply(poses.overBumpAllianceAlt))
            .withTimeout(3.5),
        rollers.stopIntakeCommand().withTimeout(0.05),
        Commands.deadline(
            new DriveToPose(drive, () -> AllianceFlipUtil.apply(poses.shootFromCorner))
                .withTimeout(2),
            orchestrator.spinUpShooter(1240)),
        Commands.parallel(
            orchestrator.spinUpShooter(1240),
            orchestrator.feedUp(),
            Commands.waitSeconds(2).andThen(intake.extendToAngleAndIntake(0.0))));
  }

  private Command ccManualAutoOverBump(boolean isA) {
    AutoPositions poses = getPoses(isA);

    return Commands.sequence(
        Commands.deadline(
            new DriveToPose(drive, () -> AllianceFlipUtil.apply(poses.intakeSimplePose))
                .withTimeout(5),
            intake.extendToAngleAndIntake(IntakeConstants.EXTEND_POS)),
        new DriveToPose(drive, () -> AllianceFlipUtil.apply(poses.overBumpAllianceAlt))
            .withTimeout(3.5),
        rollers.stopIntakeCommand().withTimeout(0.05),
        Commands.deadline(
            new DriveToPose(drive, () -> AllianceFlipUtil.apply(poses.shootFromCorner))
                .withTimeout(2),
            orchestrator.spinUpShooter(1250)),
        Commands.deadline(
            Commands.waitSeconds(5),
            orchestrator.spinUpShooter(1250),
            orchestrator.feedUp(),
            Commands.waitSeconds(2).andThen(intake.extendToAngleAndIntake(0.0))),
        rollers.stopIntakeCommand().withTimeout(0.05),
        shooter.stop(),
        new DriveToPose(drive, () -> AllianceFlipUtil.apply(poses.overBumpAllianceAlt))
            .withTimeout(2),
        Commands.deadline(
            new DriveToPose(drive, () -> AllianceFlipUtil.apply(poses.intakeSimplePose))
                .withTimeout(3.5),
            intake.extendToAngleAndIntake(IntakeConstants.EXTEND_POS)));
  }

  // Manual autos using the helper functions
  public Command ACCManuelAuto() {
    return ccManualAuto(true);
  }

  public Command BCCManuelAuto() {
    return ccManualAuto(false);
  }

  public Command ACCManuelAutoAlt() {
    return ccManualAutoAlt(true);
  }

  public Command BCCManuelAutoAlt() {
    return ccManualAutoAlt(false);
  }

  public Command ACCManuelAutoOverBump() {
    return ccManualAutoOverBump(true);
  }

  public Command BCCManuelAutoOverBump() {
    return ccManualAutoOverBump(false);
  }

  // Other manual autos
  public Command EightBalls() {
    return Commands.sequence(
        Commands.deadline(
            orchestrator.driveToHub().withTimeout(3.0), orchestrator.spinUpShooterHub()),
        Commands.parallel(orchestrator.spinUpShooterHub(), orchestrator.feedUp()));
  }
}
