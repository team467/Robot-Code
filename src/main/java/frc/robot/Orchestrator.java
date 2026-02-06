package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.utils.AllianceFlipUtil;
import frc.robot.FieldConstants.Hub;
import frc.robot.commands.auto.DriveToPose;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopperbelt.HopperBelt;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
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

  public Orchestrator(
      Drive drive, HopperBelt hopperBelt, Shooter shooter, Indexer indexer, Intake intake) {
    this.drive = drive;
    this.hopperBelt = hopperBelt;
    this.shooter = shooter;
    this.indexer = indexer;
    this.shooterLeadCompensator = new ShooterLeadCompensator(drive, shooter);
    this.intake = intake;
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
  }
  //TODO move to drive commands/shooter?
  public Command driveToHub(){
    return new DriveToPose(drive, AllianceFlipUtil.apply(Hub.nearFace.transformBy(new Transform2d(0.0, -FRONT_HUB_OFFSET, Rotation2d.fromDegrees(0)))));
  }

  public Command shootAtHub(){
    return shooter.setTargetVelocity(FRONT_HUB_SHOOTER_VELOCITY);
  }

  //Toggle
  public Command shootBallsAtDistance(){
    return Commands.none();
    //TODO: Stop drive, face reef, shoot sequence
  }
  /** created command to shoot the balls so it runs the shooter, hopperBelt and indexer */
  public Command shootBallsWithDrive() {
    return preloadBalls()
        .andThen(
            Commands.parallel(
                hopperBelt.start(),
                indexer.run(),
                shooter.setTargetDistance(() -> shooterLeadCompensator.shootWhileDriving(Hub.innerCenterPoint.toTranslation2d()).distance())))
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

  public Command alignAndShootWhileDriving(DoubleSupplier xsupplier, DoubleSupplier ysupplier) {
    return Commands.parallel(shootBallsWithDrive());
    //    shooterLeadCompensator.shootWhileDriving(Hub.innerCenterPoint.toTranslation2d()).target();
    // TODO: AIMING LOGIC
  }
}
