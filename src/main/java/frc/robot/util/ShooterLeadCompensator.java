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
            new Translation2d();

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

    /**
     * Our function for defining our target pose is dependant on the distance it takes for the ball
     * to reach the final pose. This circular reasoning is a problem. To fix it we assume that
     * distance is equal to our current "target" and adjust our target pose by the distance to the
     * current "target." After around 10 iterations we will have a very good approximation for the
     * optimal target.
     */
    for (int i = 0; i < 10; i++) {
      double distance = aimPoint.minus(shooterPos).getNorm();
      double t = 0.02;
      aimPoint = targetPosition.plus(v.times(t));
    }

    double finalDistance = aimPoint.minus(shooterPos).getNorm();
    Pose2d aimPose = new Pose2d(aimPoint, aimPoint.getAngle());

    return new ShootWhileDrivingResult(aimPose, finalDistance);
  }

  public record ShootWhileDrivingResult(Pose2d target, double distance) {}
}
