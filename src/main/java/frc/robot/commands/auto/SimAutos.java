package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.Orchestrator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.rollers.IntakeRollers;
import frc.robot.subsystems.shooter.Shooter;

public class SimAutos {
  private final Drive drive;
  private final Orchestrator orchestrator;
  private final Intake intake;
  private final IntakeRollers rollers;
  private final Shooter shooter;

  public SimAutos(
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

  private static final Pose2d startAside =
      new Pose2d(FieldConstants.fieldLength - 3.645, 5.520, new Rotation2d(Math.PI));

  private static final Pose2d centerOfField =
      new Pose2d(
          FieldConstants.fieldLength / 2.0,
          FieldConstants.fieldWidth / 2.0,
          new Rotation2d(Math.PI));

  public Command sim1() {
    return Commands.sequence(
        Commands.runOnce(() -> drive.setPose(startAside)),
        new StraightDriveToPose(drive, centerOfField));
  }
}
