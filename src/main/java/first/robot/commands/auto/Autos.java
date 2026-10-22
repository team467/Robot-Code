package first.robot.commands.auto;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Seconds;

import first.robot.Orchestrator;
import first.robot.constants.IntakeConstants;
import first.robot.mechanisms.Intake.Intake;
import first.robot.mechanisms.shooter.Shooter;
import first.robot.mechanisms.swerve.Drive;
import first.robot.util.AllianceFlipUtil;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.ParallelGroupBuilder;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.Units;

public class Autos {
  public record AutoPositions(
      Pose2d center,
      Pose2d intakeSimple,
      Pose2d overBumpNeutral,
      Pose2d overBumpAlliance,
      Pose2d overBumpAllianceAlt,
      Pose2d shootFromCorner) {}

  private final AutoPositions poseA =
      new AutoPositions(
           new Pose2d(3.457, 4.941, new Rotation2d(Units.degreesToRadians(-55.305))),
           new Pose2d(
              7.7052903175354, 5.8276801109313965, Rotation2d.fromDegrees(0.0)),
           new Pose2d(6.1, 5.574310302734375, Rotation2d.fromDegrees(0)),
           new Pose2d(
              3.0666706562042236, 5.574310302734375, Rotation2d.fromDegrees(0.0)),
           new Pose2d(
              3.086160182952881, 5.437880039215088, Rotation2d.fromDegrees(0)),
           new Pose2d(
              3.086160182952881, 5.437880039215088, Rotation2d.fromRadians(-0.7553977556351216)));

  private final AutoPositions poseB =
      new AutoPositions(
          AllianceFlipUtil.reflectY(poseA.center),
          AllianceFlipUtil.reflectY(poseA.intakeSimple),
          AllianceFlipUtil.reflectY(poseA.overBumpNeutral),
          AllianceFlipUtil.reflectY(poseA.overBumpAlliance),
          AllianceFlipUtil.reflectY(poseA.overBumpAllianceAlt),
          AllianceFlipUtil.reflectY(poseA.shootFromCorner));

  private final Drive drive;
  private final Orchestrator orchestrator;
  private final Intake intake;
  private final Shooter shooter;

  public Autos(Drive drive, Orchestrator orchestrator, Intake intake, Shooter shooter) {
    this.drive = drive;
    this.orchestrator = orchestrator;
    this.intake = intake;
    this.shooter = shooter;
  }

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
  private static final Supplier<Pose2d> center =
      () -> AllianceFlipUtil.apply(new Pose2d(3.581, 4.027, Rotation2d.PI));

  private static Command deadline(Command main, Command... others) {
    return new ParallelGroupBuilder().requiring(main).optional(others).withAutomaticName();
  }

  private static Command parallel(Command... commands) {
    return Command.parallel(commands).withAutomaticName();
  }

  private static Command sequence(Command... commands) {
    return Command.sequence(commands).withAutomaticName();
  }

  private static Command waitSeconds(double seconds) {
    return Command.waitFor(Seconds.of(seconds)).named("Wait[" + seconds + "s]");
  }

  private static Command timeout(Command command, double seconds) {
    return command.withTimeout(Seconds.of(seconds));
  }

  private static Command after(double seconds, Command command) {
    return sequence(waitSeconds(seconds), command);
  }

  private static Command runOnce(String name, Runnable action) {
    return Command.noRequirements(coroutine -> action.run()).named(name);
  }

  private static Command repeatedly(Supplier<Command> command) {
    return Command.noRequirements(
            coroutine -> {
              while (true) {
                coroutine.await(command.get());
                coroutine.yield();
              }
            })
        .named("Repeatedly");
  }

  private Command resetPose(Supplier<Pose2d> pose) {
    return runOnce("ResetPose", () -> drive.setPose(pose.get()));
  }

  private Command ppCycle(String path, Supplier<Pose2d> startPose) {
    return sequence(resetPose(startPose), ppCycleConnect(path));
  }

  private Command ppCycleConnect(String path) {
    return sequence(
        timeout(
            deadline(
                drive.followPath(path),
                timeout(intake.extendToAngleAndIntake(IntakeConstants.EXTEND_POS), 5.5),
                orchestrator.spinUpShooterHub()),
            14.5),
        timeout(orchestrator.aimToHub(), 2.5),
        timeout(parallel(orchestrator.spinUpShooter(1215), orchestrator.feedUp()), 2.5),
        parallel(
            intake.extendToAngleAndIntake(IntakeConstants.COLLAPSE_POS),
            orchestrator.spinUpShooter(1214),
            orchestrator.feedUp()));
  }

  private Command aimAndShootRegression(double intakeDelay, double feedDelay, Double shootSeconds) {
    Command shoot =
        parallel(
            after(intakeDelay, intake.slowlyBringInIntake()),
            repeatedly(() -> timeout(orchestrator.aimToHub(), 1)),
            orchestrator.spinUpShooterDistance(orchestrator.getHubDistance()),
            after(feedDelay, orchestrator.feedUp()));
    return sequence(
        deadline(
            timeout(timeout(orchestrator.aimToHub(), 1), 2.5),
            orchestrator.spinUpShooterDistance(orchestrator.getHubDistance())),
        shootSeconds == null ? shoot : timeout(shoot, shootSeconds));
  }

  private Command ppCycleRegression(String path, Supplier<Pose2d> startPose) {
    return sequence(
        resetPose(startPose),
        timeout(
            deadline(
                drive.followPath(path),
                timeout(intake.extendToAngleAndIntake(IntakeConstants.EXTEND_POS), 5.5),
                after(5.7, orchestrator.spinUpShooterDistance(() -> Meters.of(2.769)))),
            15.5),
        aimAndShootRegression(0.4, 0.4, 1.6));
  }

  private Command ppCycleRegressionCustom(
      String path, Supplier<Pose2d> startPose, double endDistance) {
    return sequence(
        resetPose(startPose),
        timeout(
            deadline(
                drive.followPath(path),
                sequence(
                    timeout(intake.extendToAngleAndIntake(IntakeConstants.FUNNEL_POS), 4.5),
                    intake.extendToAngleAndIntake(IntakeConstants.EXTEND_POS)),
                after(5.7, orchestrator.spinUpShooterDistance(() -> Meters.of(endDistance)))),
            15.5),
        aimAndShootRegression(0.4, 0.4, 1.6));
  }

  private Command ppCycleRegressionDepo(String path) {
    return sequence(
        timeout(
            deadline(
                drive.followPath(path),
                intake.extendToAngleAndIntake(IntakeConstants.EXTEND_POS),
                orchestrator.spinUpShooterDistance(() -> Meters.of(3.185))),
            14.5),
        aimAndShootRegression(0.9, 0.9, null));
  }

  private Command ppCycleRegressionConnect(String path) {
    return sequence(
        timeout(
            deadline(
                drive.followPath(path),
                timeout(intake.extendToAngleAndIntake(IntakeConstants.EXTEND_POS), 5.5),
                after(5.5, orchestrator.spinUpShooterDistance(() -> Meters.of(2.869)))),
            14.5),
        aimAndShootRegression(0.7, 0.4, null));
  }

  private Command pp2CycleRegression(String path1, String path2, Supplier<Pose2d> startPose) {
    return sequence(
        timeout(ppCycleRegression(path1, startPose), 14.5),
        timeout(orchestrator.stopShootingAuto(), 0.1),
        ppCycleRegressionConnect(path2));
  }

  public Command ppDepot() {
    return sequence(timeout(EightBalls(), 8.2), ppCycleRegressionDepo("Depot+Outpost"));
  }

  public Command ppB2CycleRightRegression() {
    return pp2CycleRegression("B-Cycle1", "B-Cycle2", startBside);
  }

  public Command ppA2CycleRightRegression() {
    return pp2CycleRegression("A-Cycle1", "A-Cycle2", startAside);
  }

  public Command ppA1CycleRegression() {
    return ppCycleRegression("A-Cycle1", startAside);
  }

  public Command ppB1CycleRegression() {
    return ppCycleRegression("B-Cycle1", startBside);
  }

  public Command ppDepotOutpost() {
    return ppCycleRegressionCustom("Depot+Outpost", center, 4.285);
  }

  public Command followPath(String path) {
    return drive.followPath(path);
  }

  AutoPositions getPoses(boolean isA) {
    if (isA) {
      return poseA;
    }
    return poseB;
  }

  private Command driveTo(Pose2d bluePose) {
    return new DriveToPose(drive, () -> AllianceFlipUtil.apply(bluePose));
  }

  private Command ccManualAuto(boolean isA) {
    AutoPositions poses = getPoses(isA);

    return sequence(
        deadline(
            timeout(driveTo(poses.intakeSimple), 5),
            intake.extendToAngleAndIntake(IntakeConstants.EXTEND_POS)),
        timeout(driveTo(poses.overBumpAllianceAlt), 5),
        timeout(intake.stopIntakeCommand(), 0.05),
        deadline(timeout(orchestrator.driveToHub(), 3), orchestrator.spinUpShooterHub()),
        parallel(
            orchestrator.spinUpShooterHub(),
            orchestrator.feedUp(),
            after(2, intake.extendToAngleAndIntake(0.0))));
  }

  public Command ACCManualAuto() {
    return ccManualAuto(true);
  }

  public Command BCCManualAuto() {
    return ccManualAuto(false);
  }

  private Command ccManualAutoAlt(boolean isA) {
    AutoPositions poses = getPoses(isA);

    return sequence(
        deadline(
            timeout(driveTo(poses.intakeSimple), 5),
            intake.extendToAngleAndIntake(IntakeConstants.EXTEND_POS)),
        timeout(driveTo(poses.overBumpAllianceAlt), 3.5),
        timeout(intake.stopIntakeCommand(), 0.05),
        deadline(timeout(driveTo(poses.shootFromCorner), 2), orchestrator.spinUpShooter(1240)),
        parallel(
            orchestrator.spinUpShooter(1240),
            orchestrator.feedUp(),
            after(2, intake.extendToAngleAndIntake(0.0))));
  }

  public Command ACCManuelAutoAlt() {
    return ccManualAutoAlt(true);
  }

  public Command BCCManuelAutoAlt() {
    return ccManualAutoAlt(false);
  }

  private Command ccManualAutoOverBump(boolean isA) {
    AutoPositions poses = getPoses(isA);

    return sequence(
        deadline(
            timeout(driveTo(poses.intakeSimple), 5),
            intake.extendToAngleAndIntake(IntakeConstants.EXTEND_POS)),
        timeout(driveTo(poses.overBumpAllianceAlt), 3.5),
        timeout(intake.stopIntakeCommand(), 0.05),
        deadline(timeout(driveTo(poses.shootFromCorner), 2), orchestrator.spinUpShooter(1250)),
        deadline(
            waitSeconds(5),
            orchestrator.spinUpShooter(1250),
            orchestrator.feedUp(),
            after(2, intake.extendToAngleAndIntake(0.0))),
        timeout(intake.stopIntakeCommand(), 0.05),
        shooter.stop(),
        timeout(driveTo(poses.overBumpAllianceAlt), 2),
        deadline(
            timeout(driveTo(poses.intakeSimple), 3.5),
            intake.extendToAngleAndIntake(IntakeConstants.EXTEND_POS)));
  }

  public Command ACCManualAutoOverBump() {
    return ccManualAutoOverBump(true);
  }

  public Command BCCManualAutoOverBump() {
    return ccManualAutoOverBump(false);
  }

  public Command EightBalls() {
    return sequence(
        deadline(
            timeout(orchestrator.driveToHub(), 3.0),
            orchestrator.spinUpShooterDistance(orchestrator.getHubDistance())),
        timeout(
            parallel(
                orchestrator.spinUpShooterDistance(orchestrator.getHubDistance()),
                after(1, orchestrator.feedUp())),
            5.2));
  }
}
