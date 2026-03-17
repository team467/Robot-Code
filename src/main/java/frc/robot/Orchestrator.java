package frc.robot;

import static frc.robot.subsystems.shooter.ShooterConstants.CLOSE_HUB_SHOOTER_RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.utils.AllianceFlipUtil;
import frc.robot.FieldConstants.Hub;
import frc.robot.commands.auto.DriveToPose;
import frc.robot.commands.auto.RotateToOrientation;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.magicCarpet.MagicCarpet;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShotCalculator;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Orchestrator {
  private final double FRONT_HUB_OFFSET = Units.inchesToMeters(40.0);
  private final double FRONT_HUB_SHOOTER_VELOCITY = 0.0;
  private final Drive drive;
  private final Shooter shooter;
  private final MagicCarpet magicCarpet;
  private final Indexer indexer;
  private final Intake intake;
  private final RobotState robotState = RobotState.getInstance();
  private final CommandXboxController driverController;

  public Orchestrator(
      Drive drive,
      MagicCarpet hopperBelt,
      Shooter shooter,
      Indexer indexer,
      Intake intake,
      CommandXboxController driverController) {
    this.drive = drive;
    this.magicCarpet = hopperBelt;
    this.shooter = shooter;
    this.indexer = indexer;
    this.intake = intake;
    this.driverController = driverController;
  }

  /** Run the fire control solver and return the result. */
  private ShotCalculator.LaunchParameters solveShot() {
    Translation2d hubCenter = AllianceFlipUtil.apply(Hub.blueCenter);
    // Hub forward vector points from the hub face toward the robot
    Translation2d hubForward =
        new Translation2d(
            drive.getPose().getX() - hubCenter.getX(), drive.getPose().getY() - hubCenter.getY());
    double norm = hubForward.getNorm();
    if (norm > 0.01) {
      hubForward = new Translation2d(hubForward.getX() / norm, hubForward.getY() / norm);
    }

    // Convert robot-relative chassis speeds to field-relative for the solver
    ChassisSpeeds robotSpeeds = drive.getChassisSpeeds();
    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, drive.getRotation());

    return shooter.solve(
        drive.getPose(),
        fieldSpeeds,
        robotSpeeds,
        hubCenter,
        hubForward,
        1.0); // TODO: pass actual vision confidence when available
  }

  public Pose2d getShootWhileDrivingResultPose() {
    ShotCalculator.LaunchParameters params = solveShot();
    if (params.isValid()) {
      return new Pose2d(drive.getPose().getTranslation(), params.driveAngle());
    }
    // Fallback: aim directly at hub
    Translation2d hubCenter = AllianceFlipUtil.apply(Hub.blueCenter);
    return new Pose2d(hubCenter.getX(), hubCenter.getY(), Rotation2d.fromDegrees(0));
  }

  public double getShootWhileDrivingResultDistance() {
    ShotCalculator.LaunchParameters params = solveShot();
    if (params.isValid()) {
      return params.solvedDistanceM();
    }
    // Fallback: straight-line distance
    return AllianceFlipUtil.apply(Hub.blueCenter).getDistance(drive.getPose().getTranslation());
  }

  public void orchestratorPeriodic() {
    ShotCalculator.LaunchParameters params = solveShot();
    if (params.isValid()) {
      Logger.recordOutput(
          "Orchestrator/Target", new Pose2d(drive.getPose().getTranslation(), params.driveAngle()));
      Logger.recordOutput("Orchestrator/Confidence", params.confidence());
      Logger.recordOutput("Orchestrator/SolvedRPM", params.rpm());
    }
  }

  // TODO move to drive commands/shooter?
  public Command driveToHub() {
    return new DriveToPose(
        drive,
        () -> {
          Pose2d hubApproachPose =
              AllianceFlipUtil.apply(
                  Hub.nearFace.transformBy(
                      new Transform2d(FRONT_HUB_OFFSET, 0.0, Rotation2d.fromDegrees(0.0))));
          Rotation2d sameAsAlignAndShootHeading =
              AllianceFlipUtil.apply(Hub.blueCenter)
                  .minus(hubApproachPose.getTranslation())
                  .getAngle();
          return new Pose2d(hubApproachPose.getTranslation(), sameAsAlignAndShootHeading);
        });
  }

  public Command alignToHub() {
    return new RotateToOrientation(
        drive,
        () -> {
          Pose2d hubApproachPose = AllianceFlipUtil.apply(Hub.nearFace);
          Rotation2d angle =
              hubApproachPose.getTranslation().minus(drive.getPose().getTranslation()).getAngle();
          return angle;
        });
  }

  public Command feedUp() {
    return Commands.repeatingSequence(
            preloadBalls().until(() -> RobotState.getInstance().shooterAtSpeed),
            Commands.parallel(magicCarpet.run(), indexer.run())
                .until(() -> !RobotState.getInstance().shooterAtSpeed))
        .onlyIf(() -> shooter.getSetpoint() > 0)
        .onlyWhile(() -> shooter.getSetpoint() > 0)
        .withName("feedUp");
  }

  public Command shootBallsDistance(DoubleSupplier targetDistance) {
    return Commands.parallel(spinUpShooterDistance(targetDistance), feedUp())
        .withName("shootBallsDistance");
  }

  public Command spinUpShooterDistance(DoubleSupplier targetDistance) {
    return shooter.setTargetVelocityRadians(() -> shooter.calculateSetpoint(targetDistance));
  }

  public Command spinUpShooterHub() {
    return shooter.setTargetVelocityRadians(
        Units.rotationsPerMinuteToRadiansPerSecond(CLOSE_HUB_SHOOTER_RPM));
  }

  public Command spinUpShooter(double velocityRPM) {
    return shooter.setTargetVelocityRadians(
        Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM));
  }

  public Command preloadBalls() {
    return Commands.parallel(magicCarpet.run(), indexer.runPreloadSpeeds())
        .onlyWhile(() -> !RobotState.getInstance().indexerHasFuel)
        .onlyIf(() -> !RobotState.getInstance().indexerHasFuel)
        .until(() -> RobotState.getInstance().indexerHasFuel)
        .andThen(indexer.stop().withTimeout(0.01));
  }

  /** Shoot-on-the-move: spins up to fire-control-solved RPM and aims at the compensated heading. */
  public Command shootOnTheMove() {
    return Commands.parallel(
        shooter.setTargetVelocityRadians(
            () -> {
              ShotCalculator.LaunchParameters params = solveShot();
              if (params.isValid() && params.confidence() > 30) {
                return params.rpm() * (2.0 * Math.PI) / 60.0;
              }
              return 0.0;
            }),
        DriveCommands.joystickDriveAtAngle(
            drive,
            driverController::getLeftX,
            driverController::getLeftY,
            () -> {
              ShotCalculator.LaunchParameters params = shooter.getLastLaunchParams();
              if (params.isValid()) {
                return params.driveAngle();
              }
              return AllianceFlipUtil.apply(Hub.blueCenter)
                  .minus(drive.getPose().getTranslation())
                  .getAngle();
            }),
        feedUp());
  }

  public Command driveShootAtAngle() {
    return Commands.parallel(
        Commands.run(
            () -> {
              shootBallsDistance(this::getShootWhileDrivingResultDistance);
            }),
        DriveCommands.joystickDriveAtAngle(
            drive,
            driverController::getLeftX,
            driverController::getLeftY,
            () ->
                (getShootWhileDrivingResultPose()
                    .getTranslation()
                    .minus(drive.getPose().getTranslation())
                    .getAngle())));
  }

  public Command spinUpShooterWhileDriving() {
    return shooter.setTargetVelocityRadians(
        () -> shooter.calculateSetpoint(this::getShootWhileDrivingResultDistance));
  }

  public Command alignAndShoot() {
    return Commands.sequence(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> 0.0,
                () -> 0.0,
                () ->
                    AllianceFlipUtil.apply(Hub.blueCenter)
                        .minus(drive.getPose().getTranslation())
                        .getAngle()
                        .plus(Rotation2d.fromDegrees(0.0))),
            shootBallsDistance(
                () ->
                    AllianceFlipUtil.apply(Hub.blueCenter)
                        .getDistance(drive.getPose().getTranslation())))
        .repeatedly();
  }
}
