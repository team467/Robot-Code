package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.Hub;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopperbelt.HopperBelt;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;

public class Orchestrator {
  private final Drive drive;
  private final Shooter shooter;
  private final HopperBelt hopperBelt;
  private final Indexer indexer;
  private final RobotState robotState = RobotState.getInstance();

  public Orchestrator(Drive drive, HopperBelt hopperBelt, Shooter shooter, Indexer indexer) {
    this.drive = drive;
    this.hopperBelt = hopperBelt;
    this.shooter = shooter;
    this.indexer = indexer;
  }

  /** created command to shoot the balls so it runs the shooter, hopperBelt and indexer */
  public Command shootBalls() {
    return Commands.parallel(shooter.setVoltage(1), hopperBelt.start(), indexer.run());
  }

  public double getAngleToShoot() {
    double robotAngle = drive.getPose().getRotation().getRadians();
    Translation2d hubPose = Hub.topCenterPoint.toTranslation2d();
    Translation2d robotPose = drive.getPose().getTranslation();
    Translation2d robotVelocity = new Translation2d(drive.getChassisSpeeds().vxMetersPerSecond,
        drive.getChassisSpeeds().vyMetersPerSecond);
    Translation2d finalTarget = hubPose.minus(projection(hubPose.minus( robotPose), robotVelocity));
    double angleDifference = Math.acos(finalTarget.dot(robotPose) / (finalTarget.getNorm()*robotPose.getNorm()));
    return robotAngle + angleDifference;
  }
  public double getDistanceToShoot() {
    Translation2d hubPose = Hub.topCenterPoint.toTranslation2d();
    Translation2d robotPose = drive.getPose().getTranslation();
    Translation2d robotVelocity = new Translation2d(drive.getChassisSpeeds().vxMetersPerSecond,
        drive.getChassisSpeeds().vyMetersPerSecond);
    Translation2d finalTarget = hubPose.minus(projection(hubPose.minus( robotPose), robotVelocity));
    return robotPose.minus(finalTarget).getNorm();
  }

  public Translation2d projection(Translation2d a, Translation2d b){
    double scalar = (a.dot(b)) / (b.dot(b));
    return new Translation2d(b.getX() * scalar, b.getY() * scalar);
  }
}
