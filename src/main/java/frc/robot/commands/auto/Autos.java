package frc.robot.commands.auto;

import static org.wpilib.units.Units.Seconds;

import frc.lib.utils.AllianceFlipUtil;
import frc.robot.Robot;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.Units;

/**
 * Contains all autos
 */
public class Autos {

  private static final Supplier<Pose2d> startAside =
      () -> AllianceFlipUtil.apply(new Pose2d(4.295, 7.531, new Rotation2d(0.000)));
  private static final Supplier<Pose2d> startBside =
      () -> AllianceFlipUtil.apply(new Pose2d(4.295, 0.538, new Rotation2d(0.000)));
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

  // pp
  private final Robot r;

  /**
   * Basic constructor, takes in all initialized subsystems and stores them
   *
   */
  public Autos(
      Robot r) {
    this.r = r;
  }

  /**
   * Helper function for a single cycle auto that uses shooter regression to shoot into the hub
   *
   * @param path The pathplanner path file name to use
   * @return A command that follows the path and shoots
   */
  private Command ppCycleRegression(String path) {
    return Command.noRequirements(
        coro -> {
          coro.awaitAny(
              r.followPath(path),
              Command.noRequirements(
                  coro2 -> {
                    r.intake();
                    coro2.wait(Seconds.of(5));
                    r.shootManual();
                    while (true) {
                      coro2.yield();
                    }
                  }
              ).whenCanceled(() -> {
                r.stopIntaking();
                r.stopShooting();
              }).named("Auto Intake & Shooter Spinup")
          );

          coro.await(
              Command.noRequirements(
                  coro2 -> {
                    r.shoot();
                    coro2.wait(Seconds.of(2.6));
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
          r.initPose(startPose.get());
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
   * Center auto that shoots the 8 balls we preload with and nothing else
   *
   * @return A command that calls the auto
   */
  public Command eightBalls() {
    return Command.noRequirements(coro -> {
      r.shoot();

      coro.wait(Seconds.of(3));

      r.stopShooting();
    }).named("Eight Balls Auto");
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
