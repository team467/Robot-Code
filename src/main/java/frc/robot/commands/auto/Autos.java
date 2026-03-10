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
import java.util.function.Supplier;

public class Autos {
  private final Drive drive;
  private final Orchestrator orchestrator;
  private final Intake intake;

  public Autos(Drive drive, Orchestrator orchestrator, Intake intake) {
    this.drive = drive;
    this.orchestrator = orchestrator;
    this.intake = intake;
  }

  private static final Supplier<Pose2d> climb =
      () -> new Pose2d(1.583, 3.750, new Rotation2d(Units.degreesToRadians(180.000)));
  private static final Supplier<Pose2d> center = () -> new Pose2d(3.504, 4.019, new Rotation2d(0));
  private static final Supplier<Pose2d> CenterA =
      () -> new Pose2d(3.457, 4.941, new Rotation2d(Units.degreesToRadians(-55.305)));
  private static final Supplier<Pose2d> firstPoseA =
      () -> new Pose2d(7.7052903175354, 5.8276801109313965, Rotation2d.fromDegrees(0.0));
  private static final Supplier<Pose2d> secondPoseA =
      () -> new Pose2d(3.0666706562042236, 5.808189868927002, Rotation2d.fromDegrees(0.0));
  private static final Supplier<Pose2d> firstPoseB =
      () -> new Pose2d(7.724780082702637, 2.436420202255249, Rotation2d.fromDegrees(0.0));
  private static final Supplier<Pose2d> secondPoseB =
      () -> new Pose2d(3.1251401901245117, 2.397440195083618, Rotation2d.fromDegrees(0.0));
  private static final Supplier<Pose2d> thirdPose =
      () -> new Pose2d(0.5134801864624023, 0.6628301739692688, Rotation2d.fromDegrees(180));

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

  public Command EightBalls() {
    return Commands.sequence(
        Commands.deadline(
            orchestrator.driveToHub().withTimeout(3.0), orchestrator.spinUpShooterHub()),
        Commands.parallel(orchestrator.spinUpShooterHub(), orchestrator.feedUp()));
  }

  public Command ACCManuelAuto() {
    return Commands.sequence(
        Commands.deadline(
            new DriveToPose(drive, () -> AllianceFlipUtil.apply(firstPoseA.get())).withTimeout(5),
            Commands.parallel(
                intake.holdAngleAndIntake(IntakeConstants.EXTEND_POS),
                orchestrator.preloadBalls())),
        new DriveToPose(drive, () -> AllianceFlipUtil.apply(secondPoseA.get())).withTimeout(5),
        intake.stopIntakeCommand().withTimeout(0.05),
        Commands.deadline(
            orchestrator.driveToHub().withTimeout(3), orchestrator.spinUpShooterHub()),
        Commands.parallel(
            orchestrator.spinUpShooterHub(),
            orchestrator.feedUp(),
            Commands.waitSeconds(2).andThen(intake.extendToAngleAndIntake(0.0))),
        intake.stopIntakeCommand(),
        Commands.deadline(
            new DriveToPose(drive, () -> AllianceFlipUtil.apply(thirdPose.get())).withTimeout(5),
            Commands.parallel(
                intake.extendToAngle(IntakeConstants.EXTEND_POS), orchestrator.preloadBalls())));
  }

  public Command BCCManuelAuto() {
    return Commands.sequence(
        Commands.deadline(
            new DriveToPose(drive, () -> AllianceFlipUtil.apply(firstPoseB.get())).withTimeout(5),
            Commands.parallel(
                intake.holdAngleAndIntake(IntakeConstants.EXTEND_POS),
                orchestrator.preloadBalls())),
        new DriveToPose(drive, () -> AllianceFlipUtil.apply(secondPoseB.get())).withTimeout(5),
        intake.stopIntakeCommand().withTimeout(0.05),
        Commands.deadline(
            orchestrator.driveToHub().withTimeout(3), orchestrator.spinUpShooterHub()),
        Commands.parallel(
            orchestrator.spinUpShooterHub(),
            orchestrator.feedUp(),
            Commands.waitSeconds(2).andThen(intake.extendToAngleAndIntake(0.0))),
        intake.stopIntakeCommand(),
        Commands.deadline(
            new DriveToPose(drive, () -> AllianceFlipUtil.apply(thirdPose.get())).withTimeout(5),
            Commands.parallel(
                intake.extendToAngle(IntakeConstants.EXTEND_POS), orchestrator.preloadBalls())));
  }
}
