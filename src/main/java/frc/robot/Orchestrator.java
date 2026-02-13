package frc.robot;

import static frc.robot.FieldConstants.Hub;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.utils.AllianceFlipUtil;
import frc.robot.FieldConstants.Hub;
import frc.robot.commands.auto.DriveToPose;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopperbelt.HopperBelt;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.ShooterLeadCompensator;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Orchestrator {
  private final double FRONT_HUB_OFFSET = -1.0;
  private final double FRONT_HUB_SHOOTER_VELOCITY = 0.0;
  private final Drive drive;
  private final Shooter shooter;
  private final HopperBelt hopperBelt;
  private final Indexer indexer;
  private final Intake intake;
  private final RobotState robotState = RobotState.getInstance();
  private final ShooterLeadCompensator shooterLeadCompensator;
  private final CommandXboxController driverController;

  public Orchestrator(
      Drive drive,
      HopperBelt hopperBelt,
      Shooter shooter,
      Indexer indexer,
      Intake intake,
      CommandXboxController driverController) {
    this.drive = drive;
    this.hopperBelt = hopperBelt;
    this.shooter = shooter;
    this.indexer = indexer;
    this.intake = intake;
    this.shooterLeadCompensator = new ShooterLeadCompensator(drive, shooter);
    this.driverController = driverController;
  }

  public void OrchestratorPeriodic() {
    var shootWhileDrivingResult =
        shooterLeadCompensator.shootWhileDriving(
            AllianceFlipUtil.apply(Hub.innerCenterPoint.toTranslation2d()));
    Logger.recordOutput(
        "Orchestrator/Target",
        new Pose2d(
            shootWhileDrivingResult.target().getX(),
            shootWhileDrivingResult.target().getY(),
            Rotation2d.fromDegrees(0)));

    Logger.recordOutput("Shooter/DistanceToHub", drive.getPose().getTranslation().getDistance(AllianceFlipUtil.apply(Hub.topCenterPoint).toTranslation2d().plus(
        ShooterConstants.kShooterOffsetFromRobotCenter.getTranslation())));
  }

  // TODO move to drive commands/shooter?
  public Command driveToHub() {
    return new DriveToPose(
        drive,
        AllianceFlipUtil.apply(
            Hub.nearFace.transformBy(
                new Transform2d(0.0, -FRONT_HUB_OFFSET, Rotation2d.fromDegrees(0)))));
  }

  public Command shootAtHub() {
    return shootBallsVelocity(FRONT_HUB_SHOOTER_VELOCITY);
  }

  public Command shootBallsVelocity(double targetVelocity) {
    return preloadBalls()
        .andThen(shooter.setTargetVelocity(targetVelocity)) // change
        .onlyIf(() -> shooter.getSetpoint() == 0)
        .andThen(Commands.parallel(hopperBelt.start(), indexer.run()))
        .onlyWhile(
            () ->
                shooter.getSetpoint() > 0.1
                    && RobotState.getInstance().shooterAtSpeed
                    && RobotState.getInstance().isAlignedToHub)
        .finallyDo(() -> Commands.parallel(hopperBelt.stop(), preloadBalls()));
  }

  public Command shootBallsDistance(DoubleSupplier targetDistance) {
    return preloadBalls()
        .andThen(shooter.setTargetDistance(targetDistance)) // change
        .onlyIf(() -> shooter.getSetpoint() == 0)
        .andThen(Commands.parallel(hopperBelt.start(), indexer.run()))
        .onlyWhile(
            () ->
                shooter.getSetpoint() > 0.1
                    && RobotState.getInstance().shooterAtSpeed
                    && RobotState.getInstance().isAlignedToHub)
        .finallyDo(() -> Commands.parallel(hopperBelt.stop(), preloadBalls()));
  }

  /** created command to shoot the balls so it runs the shooter, hopperBelt and indexer */
  public Command shootBallsAtPoint() {
    return preloadBalls()
        .andThen(
            Commands.parallel(
                hopperBelt.start(),
                indexer.run(),
                shooter.setTargetDistance(
                    () ->
                        shooterLeadCompensator
                            .shootWhileDriving(Hub.innerCenterPoint.toTranslation2d())
                            .distance())))
        .onlyWhile(
            () ->
                shooter.getSetpoint() > 0.1
                    && RobotState.getInstance().shooterAtSpeed
                    && RobotState.getInstance().isAlignedToHub)
        .finallyDo(() -> Commands.parallel(hopperBelt.stop(), preloadBalls()));
  }

  public Command preloadBalls() {
    return indexer.run().until(indexer::isSwitchPressed);
  }

  public Command driveShootAtAngle() {
    return Commands.none();
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
                    .getAngle()),
        shootBallsDistance(
            () ->
                AllianceFlipUtil.apply(Hub.blueCenter)
                    .getDistance(drive.getPose().getTranslation())));
  }

  public Command alignAndShootWhileDriving() {
    return Commands.parallel(
        DriveCommands.joystickDriveAtAngle(
            drive,
            driverController::getLeftX,
            driverController::getLeftY,
            () ->
                shooterLeadCompensator
                    .shootWhileDriving(Hub.innerCenterPoint.toTranslation2d())
                    .target()
                    .getTranslation()
                    .minus(drive.getPose().getTranslation())
                    .getAngle()),
        shootBallsDistance(
            () ->
                shooterLeadCompensator
                    .shootWhileDriving(Hub.innerCenterPoint.toTranslation2d())
                    .distance()));
  }
}
