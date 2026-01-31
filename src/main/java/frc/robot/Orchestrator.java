package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.utils.AllianceFlipUtil;
import frc.robot.FieldConstants.Hub;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopperbelt.HopperBelt;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

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

  public void OrchestratorPeriodic() {
    Logger.recordOutput(
        "Orchestrator/Target",
        new Pose2d(
            getFinalTargetPose().getX(), getFinalTargetPose().getY(), Rotation2d.fromDegrees(0)));
    Logger.recordOutput("Orchestrator/Hub", Hub.innerCenterPoint);
    Logger.recordOutput("Orchestrator/CanShoot", canShoot());
    Logger.recordOutput("Orchestrator/ShooterVelocity", getShooterVelocity());
  }

  /** created command to shoot the balls so it runs the shooter, hopperBelt and indexer */
  public Command shootBalls() {
    return Commands.parallel(shooter.setVoltage(1), hopperBelt.start(), indexer.run());
  }

  public Command driveShootAtAngle(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return DriveCommands.joystickDriveAtAngle(drive, xSupplier, ySupplier, this::getAngleToShoot);
  }

  // FOR SHOOTING WHILE DRIVE WE MUST KNOW A FUNCTION OF SHOOTER VELOCITY TO INITIAL VELOCITY OF THE
  // BALL FORWARDS

  private Rotation2d getAngleToShoot() {
    Translation2d robotPose = drive.getPose().getTranslation();
    Translation2d finalTarget = getFinalTargetPose();
    return finalTarget.minus(robotPose).getAngle();
  }

  private double getShooterVelocity() {
    Translation2d robotPos = drive.getPose().getTranslation();
    Translation2d finalTarget = getFinalTargetPose();

    Optional<Double> shooterVelocityOpt =
        solveShooterVelocity(
            robotPos, finalTarget, Math.toRadians(ShooterConstants.SHOOTER_ANGLE_DEGREES), 9.81);

    if (!shooterVelocityOpt.isPresent()) return 0.0;

    double vRequired = shooterVelocityOpt.get();

    // subtract radial component of robot velocity
    Translation2d toHub = finalTarget.minus(robotPos);
    Translation2d robotVelocity =
        new Translation2d(
            drive.getChassisSpeeds().vxMetersPerSecond, drive.getChassisSpeeds().vyMetersPerSecond);
    Translation2d radialComponent = project(robotVelocity, toHub);

    double radialSpeed = radialComponent.getNorm();
    // check if robot is moving toward or away from hub
    if (radialComponent.dot(toHub) < 0) radialSpeed *= -1;

    double correctedShooterVelocity = vRequired - radialSpeed;
    return Math.max(correctedShooterVelocity, 0.0);
  }

  private Translation2d getFinalTargetPose() {
    Translation2d hubPose = AllianceFlipUtil.apply(Hub.topCenterPoint.toTranslation2d());
    Translation2d robotPose = drive.getPose().getTranslation();

    // Robot velocity as Translation2d
    Translation2d robotVelocity =
        new Translation2d(
            drive.getChassisSpeeds().vxMetersPerSecond, drive.getChassisSpeeds().vyMetersPerSecond);

    Translation2d toHub = hubPose.minus(robotPose);

    Translation2d radialComponent = project(robotVelocity, toHub);
    Translation2d tangentialVelocity = robotVelocity.minus(radialComponent);

    Optional<Double> shooterVelocityOpt =
        solveShooterVelocity(
            robotPose, hubPose, Math.toRadians(ShooterConstants.SHOOTER_ANGLE_DEGREES), 9.81);

    if (!shooterVelocityOpt.isPresent()) {
      return hubPose;
    }

    double shooterVelocity = shooterVelocityOpt.get();

    double flightTime =
        toHub.getNorm()
            / (shooterVelocity * Math.cos(Math.toRadians(ShooterConstants.SHOOTER_ANGLE_DEGREES)));
    Translation2d offset =
        DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)
            ? tangentialVelocity.times(flightTime).times(-1.0)
            : tangentialVelocity.times(flightTime);

    return hubPose.plus(offset);
  }

  private Translation2d project(Translation2d a, Translation2d b) {
    double scale = a.dot(b) / b.dot(b);
    return b.times(scale);
  }

  private boolean canShoot() {
    Translation2d robotPose = drive.getPose().getTranslation();
    Translation2d targetPose = getFinalTargetPose();

    Optional<Double> shooterVelocity =
        solveShooterVelocity(
            robotPose, targetPose, Math.toRadians(ShooterConstants.SHOOTER_ANGLE_DEGREES), 9.81);

    return shooterVelocity.isPresent();
  }

  private double getDistanceToShoot() {
    Translation2d robotPose = drive.getPose().getTranslation();
    Translation2d finalTarget = getFinalTargetPose();
    return robotPose.getDistance(finalTarget);
  }

  private Optional<Double> solveShooterVelocity(
      Translation2d shooterPos, Translation2d targetPos, double shooterAngleRad, double g) {

    double distance = shooterPos.getDistance(targetPos);

    double deltaZ = Hub.height - ShooterConstants.SHOOTER_HEIGHT;

    double cosTheta = Math.cos(shooterAngleRad);

    if (Math.abs(cosTheta) < 1e-6) {
      return Optional.empty();
    }

    double denom = distance * Math.tan(shooterAngleRad) - deltaZ;
    if (denom <= 0) {
      return Optional.empty();
    }

    double velocity = Math.sqrt((g * distance * distance) / (2 * cosTheta * cosTheta * denom));

    return Double.isFinite(velocity) && velocity > 0 ? Optional.of(velocity) : Optional.empty();
  }
}
