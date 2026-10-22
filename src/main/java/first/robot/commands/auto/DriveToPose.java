package first.robot.commands.auto;

import first.robot.Constants;
import first.robot.mechanisms.swerve.Drive;
import first.robot.util.AllianceFlipUtil;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.util.Units;

public class DriveToPose implements Command {
  private final Drive drive;
  private final Supplier<Pose2d> poseSupplier;

  private boolean running = false;
  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          driveKp.get(),
          0.0,
          driveKd.get(),
          new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()));
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          thetaKp.get(),
          0.0,
          thetaKd.get(),
          new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()));
  private double driveErrorAbs;
  private double thetaErrorAbs;

  private static final double THETA_TOLERANCE_DEG =
      Constants.getRobot() == Constants.RobotType.ROBOT_2026_COMP ? 0.2 : 1.0;
  private static final LoggedNetworkNumber driveKp = tunable("DriveKp", 2.5);
  private static final LoggedNetworkNumber driveKd = tunable("DriveKd", 0.0);
  private static final LoggedNetworkNumber thetaKp = tunable("ThetaKp", 3.0);
  private static final LoggedNetworkNumber thetaKd = tunable("ThetaKd", 0.01);
  private static final LoggedNetworkNumber driveMaxVelocity =
      tunable("DriveMaxVelocity", Units.inchesToMeters(85));
  private static final LoggedNetworkNumber driveMaxAcceleration =
      tunable("DriveMaxAcceleration", Units.inchesToMeters(450.0));
  private static final LoggedNetworkNumber thetaMaxVelocity =
      tunable("ThetaMaxVelocity", Units.degreesToRadians(360.0));
  private static final LoggedNetworkNumber thetaMaxAcceleration =
      tunable("ThetaMaxAcceleration", Units.degreesToRadians(720.0));
  private static final LoggedNetworkNumber driveTolerance = tunable("DriveTolerance", 0.01);
  private static final LoggedNetworkNumber thetaTolerance =
      tunable("ThetaTolerance", Units.degreesToRadians(THETA_TOLERANCE_DEG));

  private static LoggedNetworkNumber tunable(String name, double defaultValue) {
    return new LoggedNetworkNumber("/Tuning/DriveToPose/" + name, defaultValue);
  }

  public DriveToPose(Drive drive, Pose2d pose) {
    this(drive, () -> pose);
  }

  public DriveToPose(Drive drive, double deltaXMeters, double deltaYMeters, double deltaThetaRad) {
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
        });
  }

  public DriveToPose(Drive drive, Supplier<Pose2d> poseSupplier) {
    this.drive = drive;
    this.poseSupplier = poseSupplier;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  private void applyTunables() {
    driveController.setP(driveKp.get());
    driveController.setD(driveKd.get());
    driveController.setConstraints(
        new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()));
    driveController.setTolerance(driveTolerance.get());
    thetaController.setP(thetaKp.get());
    thetaController.setD(thetaKd.get());
    thetaController.setConstraints(
        new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()));
    thetaController.setTolerance(thetaTolerance.get());
  }

  @Override
  public void run(Coroutine coroutine) {
    applyTunables();
    var currentPose = drive.getPose();
    driveController.reset(
        currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation()));
    thetaController.reset(currentPose.getRotation().getRadians());

    while (true) {
      running = true;
      applyTunables();

      currentPose = drive.getPose();
      var targetPose = poseSupplier.get();

      double currentDistance =
          currentPose.getTranslation().getDistance(targetPose.getTranslation());
      driveErrorAbs = currentDistance;
      double driveVelocityScalar = driveController.calculate(driveErrorAbs, 0.0);
      if (driveController.atGoal()) driveVelocityScalar = 0.0;

      double thetaVelocity =
          thetaController.calculate(
              currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
      thetaErrorAbs =
          Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
      if (thetaController.atGoal()) thetaVelocity = 0.0;

      if (atGoal()) {
        break;
      }

      Rotation2d awayFromTarget =
          currentPose
              .getTranslation()
              .minus(targetPose.getTranslation())
              .getAngle()
              .orElse(Rotation2d.ZERO);
      var driveVelocity =
          new Pose2d(Translation2d.ZERO, awayFromTarget)
              .transformBy(new Transform2d(driveVelocityScalar, 0.0, Rotation2d.ZERO))
              .getTranslation();
      drive.runVelocity(
          new ChassisVelocities(driveVelocity.getX(), driveVelocity.getY(), thetaVelocity)
              .toRobotRelative(currentPose.getRotation()));
      Logger.recordOutput("DriveToPose/Running", true);
      coroutine.yield();
    }
    end();
  }

  @Override
  public void onCancel() {
    end();
  }

  private void end() {
    running = false;
    Logger.recordOutput("DriveToPose/Running", false);
    drive.stop();
  }

  public boolean atGoal() {
    return running && driveController.atGoal() && thetaController.atGoal();
  }

  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return running
        && Math.abs(driveErrorAbs) < driveTolerance
        && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
  }

  public boolean isRunning() {
    return running;
  }

  @Override
  public String name() {
    return "DriveToPose";
  }

  @Override
  public Set<Mechanism> requirements() {
    return Set.of(drive);
  }
}
