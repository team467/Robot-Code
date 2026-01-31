package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.Hub;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopperbelt.HopperBelt;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.Optional;

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

  //FOR SHOOTING WHILE DRIVE WE MUST KNOW A FUNCTION OF SHOOTER VELOCITY TO INITIAL VELOCITY OF THE BALL FORWARDS

  public double getAngleToShoot() {
    double robotAngle = drive.getPose().getRotation().getRadians();
    Translation2d robotPose = drive.getPose().getTranslation();
    Translation2d finalTarget = getFinalTargetPose();
    double angleDifference = Math.acos(finalTarget.dot(robotPose) / (finalTarget.getNorm()*robotPose.getNorm()));
    return robotAngle + angleDifference;
  }
  public double getShooterVelocity(){
    Translation2d robotPose = drive.getPose().getTranslation();
    Translation2d robotVelocity = new Translation2d(drive.getChassisSpeeds().vxMetersPerSecond,
        drive.getChassisSpeeds().vyMetersPerSecond);
    Translation2d finalTarget = getFinalTargetPose();
    Optional<Double> shooterVelocity = solveShooterVelocity(
        robotPose,
        finalTarget,
        Math.toRadians(45),
        robotVelocity.getNorm(),
        9.81
    );
    return shooterVelocity.orElse(0.0);
  }

  public Translation2d getFinalTargetPose(){
    Translation2d hubPose = Hub.topCenterPoint.toTranslation2d();
    Translation2d robotPose = drive.getPose().getTranslation();
    Translation2d robotVelocity = new Translation2d(drive.getChassisSpeeds().vxMetersPerSecond,
        drive.getChassisSpeeds().vyMetersPerSecond);
    return hubPose.minus(projection(hubPose.minus( robotPose), robotVelocity));
  }
  public boolean canShoot(){
    Translation2d robotPose = drive.getPose().getTranslation();
    Translation2d robotVelocity = new Translation2d(drive.getChassisSpeeds().vxMetersPerSecond,
        drive.getChassisSpeeds().vyMetersPerSecond);
    Translation2d targetPose = getFinalTargetPose();
    Optional<Double> shooterVelocity = solveShooterVelocity(
        robotPose,
        targetPose,
        Math.toRadians(ShooterConstants.SHOOTER_ANGLE_DEGREES),
        robotVelocity.getNorm(),
        9.81
    );
    return shooterVelocity.isPresent();
  }
  public double getDistanceToShoot() {
    Translation2d robotPose = drive.getPose().getTranslation();
    Translation2d finalTarget = getFinalTargetPose();
    return robotPose.minus(finalTarget).getNorm();
  }

  public Translation2d projection(Translation2d a, Translation2d b){
    double scalar = (a.dot(b)) / (b.dot(b));
    return new Translation2d(b.getX() * scalar, b.getY() * scalar);
  }

  public Optional<Double> solveShooterVelocity(
      Translation2d shooterPos,
      Translation2d targetPos,
      double shooterAngleRad,
      double vDrive,
      double g
  ) {
    double distance = shooterPos.getDistance(targetPos);
    double deltaZ = targetPos.getY() - shooterPos.getY();
    double cosTheta = Math.cos(shooterAngleRad);
    double sinTheta = Math.sin(shooterAngleRad);

    if (Math.abs(cosTheta) < 1e-6) {
      return Optional.empty();
    }

    double velocityShooterOut = (distance - vDrive) / cosTheta;
    boolean feasible = velocityShooterOut > 0 && (sinTheta * velocityShooterOut - deltaZ) >= 0;
    if (!feasible) {
      return Optional.empty();
    }
    return Optional.of(velocityShooterOut);
  }
}
