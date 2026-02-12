package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.function.Supplier;

public class ShooterLeadCompensator {

  private final Supplier<Translation2d> shooterPosition;
  private final Supplier<Translation2d> robotVelocity;
  private final Shooter shooter;

  public ShooterLeadCompensator(Drive drive, Shooter shooter) {
    this.shooterPosition =
        () ->
            drive
                .getPose()
                .transformBy(ShooterConstants.kShooterOffsetFromRobotCenter)
                .getTranslation();

    this.robotVelocity =
        () ->
            new Translation2d(
                drive.getChassisSpeeds().vxMetersPerSecond,
                drive.getChassisSpeeds().vyMetersPerSecond);

    this.shooter = shooter;
  }

  public ShootWhileDrivingResult shootWhileDriving(Translation2d targetPosition) {
    Translation2d shooterPos = shooterPosition.get();
    Translation2d v = robotVelocity.get();

    Translation2d aimPoint = targetPosition;

    for (int i = 0; i < 10; i++) {
      double distance = aimPoint.minus(shooterPos).getNorm();
      double t = shooter.getAirTimeSeconds(distance);
      aimPoint = targetPosition.plus(v.times(t));
    }

    double finalDistance = aimPoint.minus(shooterPos).getNorm();
    Pose2d aimPose = new Pose2d(aimPoint, aimPoint.getAngle());

    return new ShootWhileDrivingResult(aimPose, finalDistance);
  }

  public record ShootWhileDrivingResult(Pose2d target, double distance) {}
}
