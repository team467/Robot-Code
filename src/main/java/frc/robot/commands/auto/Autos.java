package frc.robot.commands.auto;

import static org.wpilib.units.Units.Seconds;

import frc.lib.utils.AllianceFlipUtil;
import frc.robot.Orchestrator;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine.ForkResult;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.Units;

/**
 * Contains all autos
 */
public class Autos {

  private static final Supplier<Pose2d> center = () -> new Pose2d(3.504, 4.019, new Rotation2d(0));
  private static final Supplier<Pose2d> depotPoseStopPoint =
      () -> new Pose2d(1.7998201847076416, 5.983600616455078, Rotation2d.fromDegrees(180));
  private static final Supplier<Pose2d> depotPose =
      () -> new Pose2d(0.45501017570495605, 5.983600616455078, Rotation2d.fromDegrees(180));
  private static final Supplier<Pose2d> startAside =
      () -> AllianceFlipUtil.apply(new Pose2d(4.295, 7.531, new Rotation2d(0.000)));
  private static final Supplier<Pose2d> startBside =
      () -> AllianceFlipUtil.apply(new Pose2d(4.295, 0.538, new Rotation2d(0.000)));
  private static final Supplier<Pose2d> startDepot =
      () -> AllianceFlipUtil.apply(new Pose2d(3.630, 5.844, new Rotation2d(0.000)));
  // The necessary poses for autos on left side
  private final AutoPositions poseA =
      new AutoPositions(
          /* center */ new Pose2d(3.457, 4.941, new Rotation2d(Units.degreesToRadians(-55.305))),
          /* intakeSimple */ new Pose2d(
          7.7052903175354, 5.8276801109313965, Rotation2d.fromDegrees(0.0)),
          /* overBumpNeutral */ new Pose2d(6.1, 5.574310302734375, Rotation2d.fromDegrees(0)),
          /* overBumpAlliance */ new Pose2d(
          3.0666706562042236, 5.574310302734375, Rotation2d.fromDegrees(0.0)),
          /* overBumpAllianceAlt */ new Pose2d(
          3.086160182952881, 5.437880039215088, Rotation2d.fromDegrees(0)),
          /* shootFromCorner */ new Pose2d(
          3.086160182952881, 5.437880039215088, Rotation2d.fromRadians(-0.7553977556351216)));
  // The necessary poses for autos on right side
  private final AutoPositions poseB =
      new AutoPositions(
          AllianceFlipUtil.reflectY(poseA.center),
          AllianceFlipUtil.reflectY(poseA.intakeSimple),
          AllianceFlipUtil.reflectY(poseA.overBumpNeutral),
          AllianceFlipUtil.reflectY(poseA.overBumpAlliance),
          AllianceFlipUtil.reflectY(poseA.overBumpAllianceAlt),
          AllianceFlipUtil.reflectY(poseA.shootFromCorner));
  // Relevant subsystems for the autos
  private final Drive drive;
  private final Orchestrator orchestrator;
  private final Intake intake
  private final IntakeRollers rollers;

  // pp
  private final Shooter shooter;
  private final Robot r;

  /**
   * Basic constructor, takes in all initialized subsystems and stores them
   *
   * @param drive        Drive subsystem
   * @param orchestrator Orchestrator subsystem
   * @param intake       Intake subsystem
   * @param shooter      Shooter subsystem
   */
  public Autos(
      Drive drive,
      Orchestrator orchestrator,
      Intake intake,
      Shooter shooter,
  Robot r) {
    this.drive = drive;
    this.orchestrator = orchestrator;
    this.intake = intake;
    this.shooter = shooter;
    this.r = r;
  }

  /**
   * Helper function for a single cycle auto that uses shooter regression to shoot into the hub
   *
   * @param path      The pathplanner path file name to use
   * @return A command that follows the path and shoots
   */
  private Command ppCycleRegression(String path) {
    return Command.noRequirements(
        coro -> {
          coro.awaitAny(
              drive.getAutonomousCommand(path),
              Command.noRequirements(
                  coro2 -> {
                    r.intake();
                    coro2.wait(Seconds.of(5));
                    r.shootManual();
                    while (true) {
                      coro2.yield();
                    }
                  }
              ).whenCanceled(() -> {r.stopIntaking(); r.stopShooting();}).named("Auto Intake & Shooter Spinup")
          );

          coro.await(
          Command.noRequirements(
              _ -> {
                while (true) {
                  r.shoot();
                  coro.yield();
                }
              }
          ).whenCanceled(r::stopShooting).named("Shoot").withTimeout(Seconds.of(2.6)));
        }
    ).named("ppCycleRegression");
  }

  /**
   * Helper function for a double cycle auto that uses shooter regression to shoot into the hub
   * twice (once after each cycle)
   *
   * @param path1     The pathplanner path file name to use for the first cycle
   * @param path2     The pathplanner path file name to use for the second cycle
   * @param startPose The starting position of the robot for the first path and the position it ends
   *                  up in after the path
   * @return A command that follows the paths and shoots
   */
  private Command pp2CycleRegression(String path1, String path2, Supplier<Pose2d> startPose) {
    return Command.noRequirements(
        coro -> {
          drive.setPose(startPose.get());
          coro.await(ppCycleRegression(path1).withTimeout(Seconds.of(14.5)));
          r.stopShooting();
          coro.await(ppCycleRegression(path2));
        }
    ).named("PP 2 Cycle Regression");
  }

  /**
   * Left side pathplanner auto for a single cycle that supports any shooting distance
   *
   * @return A command that executes the auto
   */
  public Command ppB2CycleRightRegression() {
    return pp2CycleRegression("B-Cycle1", "B-Cycle2", startBside);
  }

  /**
   * Left side pathplanner auto for 2 cycles that support any shooting distance
   *
   * @return A comand that executes the auto
   */
  public Command ppA2CycleRightRegression() {
    return pp2CycleRegression("A-Cycle1", "A-Cycle2", startAside);
  }

  /**
   * Gets relevant poses for the autos
   *
   * @param isA Whether to get the poses for A path or B path (true for A, false for B)
   * @return The relevant poses for the calling auto
   */
  AutoPositions getPoses(boolean isA) {
    if (isA) {
      return poseA;
    }
    return poseB;
  }

  /**
   * A helper function for a manual, straightDriveToPose auto that does one cycle with support for
   * any shooting distance
   *
   * @param isA Whether to get the poses for A path or B path (true for A, false for B)
   * @return A command that calls the auto
   */
  private Command ccManualAuto(boolean isA) {
    AutoPositions poses = getPoses(isA);

    return Commands.sequence(
        Commands.deadline(
            new DriveToPose(drive, () -> AllianceFlipUtil.apply(poses.intakeSimple)).withTimeout(5),
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

  /**
   * Left side straight drive to pose auto that does one cycle with support for any shooting
   * distance
   *
   * @return A command that calls the auto
   */
  public Command ACCManualAuto() {
    return ccManualAuto(true);
  }

  /**
   * Right side straight drive to pose auto that does one cycle with support for any shooting
   * distance
   *
   * @return A command that calls the auto
   */
  public Command BCCManualAuto() {
    return ccManualAuto(false);
  }

  /**
   * A helper function for a manual, straightDriveToPose auto that does one cycle with fixed
   * shooting distance
   *
   * @param isA Whether to get the poses for A path or B path (true for A, false for B)
   * @return A command that calls the auto
   */
  private Command ccManualAutoAlt(boolean isA) {
    AutoPositions poses = getPoses(isA);

    return Commands.sequence(
        Commands.deadline(
            new DriveToPose(drive, () -> AllianceFlipUtil.apply(poses.intakeSimple)).withTimeout(5),
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

  /**
   * Left side straight drive to pose auto that does one cycle with fixed shooting distance
   *
   * @return A command that calls the auto
   */
  public Command ACCManuelAutoAlt() {
    return ccManualAutoAlt(true);
  }

  /**
   * Right side straight drive to pose auto that does one cycle with fixed shooting distance
   *
   * @return A command that calls the auto
   */
  public Command BCCManuelAutoAlt() {
    return ccManualAutoAlt(false);
  }

  /**
   * A helper function for a manual, straightDriveToPose auto that does one cycle with fixed
   * shooting distance and then goes back to pick up more balls from the center.
   *
   * @param isA Whether to get the poses for A path or B path (true for A, false for B)
   * @return A command that calls the auto
   */
  private Command ccManualAutoOverBump(boolean isA) {
    AutoPositions poses = getPoses(isA);

    return Commands.sequence(
        Commands.deadline(
            new DriveToPose(drive, () -> AllianceFlipUtil.apply(poses.intakeSimple)).withTimeout(5),
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
            new DriveToPose(drive, () -> AllianceFlipUtil.apply(poses.intakeSimple))
                .withTimeout(3.5),
            intake.extendToAngleAndIntake(IntakeConstants.EXTEND_POS)));
  }

  /**
   * Left side straight drive to pose auto that does one cycle with fixed shooting distance and then
   * goes back to pick up more balls from the center.
   *
   * @return A command that cals the auto
   */
  public Command ACCManualAutoOverBump() {
    return ccManualAutoOverBump(true);
  }

  /**
   * Right side straight drive to pose auto that does one cycle with fixed shooting distance and
   * then goes back to pick up more balls from the center.
   *
   * @return A command that cals the auto
   */
  public Command BCCManualAutoOverBump() {
    return ccManualAutoOverBump(false);
  }

  /**
   * Center auto that shoots the 8 balls we preload with and nothing else
   *
   * @return A command that calls the auto
   */
  public Command EightBalls() {
    return Commands.sequence(
        Commands.deadline(
            orchestrator.driveToHub().withTimeout(3.0), orchestrator.spinUpShooterHub()),
        Commands.parallel(orchestrator.spinUpShooterHub(), orchestrator.feedUp()));
  }

  /**
   * A helper class that tracks poses needed for manual autos. It simplifies switching between left
   * and right sides.
   *
   * @param intakeSimple        The position to intake at
   * @param overBumpAllianceAlt The position to go over the bump
   * @param shootFromCorner     The position to shoot at the hub from
   */
  public record AutoPositions(
      Pose2d center,
      Pose2d intakeSimple,
      Pose2d overBumpNeutral,
      Pose2d overBumpAlliance,
      Pose2d overBumpAllianceAlt,
      Pose2d shootFromCorner) {

  }
}
