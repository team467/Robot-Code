package first.robot.util;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Seconds;

import first.robot.mechanisms.shooter.Shooter;
import first.robot.mechanisms.shooter.ShooterConstants;
import first.robot.mechanisms.swerve.Drive;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.units.measure.Time;

public class ShooterLeadCompensator {
  private final Supplier<Translation2d> shooterPosition;
  private final Supplier<Translation2d> robotVelocity;
  private final Shooter shooter;

  public ShooterLeadCompensator(Drive drive, Shooter shooter) {
    this.shooterPosition =
        () -> {
          Pose2d robotPose = drive.getPose();
          Translation2d rotatedOffset =
              ShooterConstants.kShooterOffsetFromRobotCenter
                  .getTranslation()
                  .rotateBy(robotPose.getRotation());
          return robotPose.getTranslation().plus(rotatedOffset);
        };

    this.robotVelocity =
        () -> {
          ChassisVelocities field =
              drive.getChassisVelocities().toFieldRelative(drive.getRotation());
          return new Translation2d(field.vx, field.vy);
        };

    this.shooter = shooter;
  }

  public ShootWhileDrivingResult shootWhileDriving(Translation2d targetPosition) {
    Translation2d shooterPos = shooterPosition.get();
    Translation2d v = robotVelocity.get();

    Translation2d aimPoint = targetPosition;

    for (int i = 0; i < 10; i++) {
      double distance = aimPoint.minus(shooterPos).getNorm();
      Time t = shooter.getAirTimeSeconds(Meters.of(distance));
      aimPoint = targetPosition.minus(v.times(t.in(Seconds)));
    }

    double finalDistance = aimPoint.minus(shooterPos).getNorm();
    Pose2d aimPose = new Pose2d(aimPoint, aimPoint.getAngle().orElse(Rotation2d.ZERO));

    return new ShootWhileDrivingResult(aimPose, finalDistance);
  }

  public Pose2d shooterPose() {
    return new Pose2d(shooterPosition.get().getX(), shooterPosition.get().getY(), Rotation2d.ZERO);
  }

  public record ShootWhileDrivingResult(Pose2d target, double distance) {}
}
