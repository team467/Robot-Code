package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.Orchestrator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.rollers.IntakeRollers;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.Supplier;

public class SimAutos {
  private final Drive drive;
  private final Orchestrator orchestrator;
  private final Intake intake;
  private final Indexer indexer;
  private final IntakeRollers rollers;
  private final Shooter shooter;

  public SimAutos(
      Drive drive,
      Orchestrator orchestrator,
      Intake intake,
      IntakeRollers rollers,
      Indexer indexer,
      Shooter shooter) {
    this.drive = drive;
    this.orchestrator = orchestrator;
    this.intake = intake;
    this.rollers = rollers;
    this.indexer = indexer;
    this.shooter = shooter;
  }

  private static final Supplier<Pose2d> shootPose =
      () -> new Pose2d(2.8, 7, new Rotation2d(Math.PI * 5 / 3));

  private static final Supplier<Pose2d> centerOfField =
      () ->
          new Pose2d(
              FieldConstants.fieldLength / 2.0, FieldConstants.fieldWidth / 2.0, new Rotation2d(0));

  public Command sim1() {
    return Commands.sequence(
        Commands.race(
            intake.extendToAngleAndIntake(-2), new StraightDriveToPose(drive, centerOfField)),
        Commands.parallel(intake.extendToAngle(0), new StraightDriveToPose(drive, shootPose)),
        Commands.parallel(shooter.setTargetVelocityRadians(140), indexer.run()).withTimeout(1),
        Commands.race(
            intake.extendToAngleAndIntake(-2), new StraightDriveToPose(drive, centerOfField)),
        Commands.parallel(intake.extendToAngle(0), new StraightDriveToPose(drive, shootPose)),
        Commands.parallel(shooter.setTargetVelocityRadians(140), indexer.run()).withTimeout(1));
  }
}
