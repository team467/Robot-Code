package first.robot.commands.auto;

import first.robot.mechanisms.swerve.Drive;
import first.robot.util.AllianceFlipUtil;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile.Constraints;
import org.wpilib.math.util.Units;

public class StraightDriveToPose implements Command {
  private final Drive drive;
  private final Supplier<Pose2d> poseSupplier;
  private final double driveTolerance;

  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          3.0,
          0,
          0.1,
          new Constraints(Units.inchesToMeters(125), Units.inchesToMeters(500.00)));
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          2.5, 0, .01, new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(720.0)));

  private static final double DRIVE_TOLERANCE = 0.005;
  private static final double THETA_TOLERANCE = Units.degreesToRadians(1.0);

  public StraightDriveToPose(
      double deltaXMeters, double deltaYMeters, double deltaThetaRad, Drive drive) {
    this(deltaXMeters, deltaYMeters, deltaThetaRad, drive, DRIVE_TOLERANCE);
  }

  public StraightDriveToPose(
      double deltaXMeters,
      double deltaYMeters,
      double deltaThetaRad,
      Drive drive,
      double driveTolerance) {
    this(
        drive,
        () -> {
          double sign = AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0;
          Pose2d pose = drive.getPose();
          return new Pose2d(
              new Translation2d(
                  pose.getTranslation().getX() + sign * deltaXMeters,
                  pose.getTranslation().getY() + sign * deltaYMeters),
              new Rotation2d(pose.getRotation().getRadians() + sign * deltaThetaRad));
        },
        driveTolerance);
  }

  public StraightDriveToPose(Pose2d targetPose, Drive drive) {
    this(drive, () -> targetPose, DRIVE_TOLERANCE);
  }

  public StraightDriveToPose(Drive drive, Pose2d targetPose) {
    this(drive, () -> targetPose, DRIVE_TOLERANCE);
  }

  public StraightDriveToPose(Drive drive, Supplier<Pose2d> targetPoseSupplier) {
    this(drive, targetPoseSupplier, DRIVE_TOLERANCE);
  }

  public StraightDriveToPose(
      Drive drive, Supplier<Pose2d> targetPoseSupplier, double driveTolerance) {
    this.drive = drive;
    this.poseSupplier = targetPoseSupplier;
    this.driveTolerance = driveTolerance;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void run(Coroutine coroutine) {
    Pose2d currentPose = drive.getPose();
    Pose2d targetPose = poseSupplier.get();
    Logger.recordOutput("StraightDriveToPose/TargetPose", targetPose);
    driveController.setTolerance(driveTolerance);
    thetaController.setTolerance(THETA_TOLERANCE);
    driveController.reset(currentPose.getTranslation().getDistance(targetPose.getTranslation()));
    thetaController.reset(currentPose.getRotation().getRadians());

    while (true) {
      currentPose = drive.getPose();

      double currentDistance =
          currentPose.getTranslation().getDistance(targetPose.getTranslation());
      double driveVelocityScalar = driveController.calculate(currentDistance, 0.0);
      if (driveController.atGoal()) driveVelocityScalar = 0.0;

      double thetaVelocity =
          thetaController.calculate(
              currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
      if (thetaController.atGoal()) thetaVelocity = 0.0;

      if (driveController.atGoal() && thetaController.atGoal()) {
        break;
      }

      Rotation2d awayFromTarget =
          currentPose
              .getTranslation()
              .minus(targetPose.getTranslation())
              .getAngle()
              .orElse(Rotation2d.ZERO);
      Translation2d driveVelocity =
          new Pose2d(Translation2d.ZERO, awayFromTarget)
              .transformBy(new Transform2d(driveVelocityScalar, 0.0, Rotation2d.ZERO))
              .getTranslation();
      drive.runVelocity(
          new ChassisVelocities(driveVelocity.getX(), driveVelocity.getY(), thetaVelocity)
              .toRobotRelative(currentPose.getRotation()));
      coroutine.yield();
    }
    drive.stop();
  }

  @Override
  public void onCancel() {
    drive.stop();
  }

  @Override
  public String name() {
    return "StraightDriveToPose";
  }

  @Override
  public Set<Mechanism> requirements() {
    return Set.of(drive);
  }
}
