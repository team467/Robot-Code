package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.utils.AllianceFlipUtil;
import frc.robot.Orchestrator;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class Autos {
  private final Drive drive;
  private final Orchestrator orchestrator;

  public Autos(Drive drive, Orchestrator orchestrator) {
    this.drive = drive;
    this.orchestrator = orchestrator;
  }

  private static final Supplier<Pose2d> climb =
      () -> new Pose2d(1.583, 3.750, new Rotation2d(Units.degreesToRadians(180.000)));
  private static final Supplier<Pose2d> center = () -> new Pose2d(3.504, 4.019, new Rotation2d(0));
  private static final Supplier<Pose2d> CenterA =
      () -> new Pose2d(3.457, 4.941, new Rotation2d(Units.degreesToRadians(-55.305)));

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
}
