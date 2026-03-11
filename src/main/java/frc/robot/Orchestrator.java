package frc.robot;

import static frc.robot.subsystems.shooter.ShooterConstants.CLOSE_HUB_SHOOTER_RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
import frc.robot.util.ShooterLeadCompensator;
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
  private final ShooterLeadCompensator shooterLeadCompensator;
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
    this.shooterLeadCompensator = new ShooterLeadCompensator(drive, shooter);
    this.driverController = driverController;
  }

  public Pose2d getShootWhileDrivingResultPose() {
    var shootWhileDrivingResult =
        shooterLeadCompensator.shootWhileDriving(
            AllianceFlipUtil.apply(Hub.innerCenterPoint.toTranslation2d()));
    return new Pose2d(
        shootWhileDrivingResult.target().getX(),
        shootWhileDrivingResult.target().getY(),
        Rotation2d.fromDegrees(0));
  }

  public double getShootWhileDrivingResultDistance() {
    var shootWhileDrivingResult =
        shooterLeadCompensator.shootWhileDriving(
            AllianceFlipUtil.apply(Hub.innerCenterPoint.toTranslation2d()));
    return shootWhileDrivingResult.distance();
  }

  public void orchestratorPeriodic() {
    var shootWhileDrivingResult =
        shooterLeadCompensator.shootWhileDriving(
            AllianceFlipUtil.apply(Hub.innerCenterPoint.toTranslation2d()));
    Logger.recordOutput(
        "Orchestrator/Target",
        new Pose2d(
            shootWhileDrivingResult.target().getX(),
            shootWhileDrivingResult.target().getY(),
            Rotation2d.fromDegrees(0)));
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

  //  public Command startFlywheelAllianceShift() {
  //    return Commands.sequence(
  //        Commands.waitUntil(() -> !DriverStation.getGameSpecificMessage().isEmpty()),
  //        Commands.runOnce(
  //            () -> {
  //              Optional<Alliance> alliance = DriverStation.getAlliance();
  //              String gameData = DriverStation.getGameSpecificMessage();
  //              double gameDataTime = Timer.getMatchTime();
  //              if (alliance.isEmpty()) {
  //                System.err.print("alliance data not found");
  //                return;
  //              }
  //              double start1 = 0;
  //              double start2 = 0;
  //              if (gameData.charAt(0) == 'B') {
  //                if (alliance.get() == DriverStation.Alliance.Blue) {
  //                  start1 = 54 - gameDataTime;
  //                  start2 = 104 - gameDataTime;
  //                } else if (alliance.get() == DriverStation.Alliance.Red) {
  //                  start1 = 79 - gameDataTime;
  //                  start2 = 129 - gameDataTime;
  //                }
  //              } else if (gameData.charAt(0) == 'R') {
  //                if (alliance.get() == DriverStation.Alliance.Red) {
  //                  start1 = 54 - gameDataTime;
  //                  start2 = 104 - gameDataTime;
  //                } else if (alliance.get() == DriverStation.Alliance.Blue) {
  //                  start1 = 79 - gameDataTime;
  //                  start2 = 129 - gameDataTime;
  //                }
  //              }
  //              Commands.waitSeconds(start1).andThen(spinUpShooterWhileDriving()).isScheduled();
  //
  //              Commands.waitSeconds(start2).andThen(spinUpShooterWhileDriving()).isScheduled();
  //            }));
  //  }
}
