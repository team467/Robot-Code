package frc.robot.commands.auto;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.utils.GeomUtils;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.function.Supplier;

public class BezierDriveToPose extends Command {
  private final Drive drive;
  private final Supplier<Pose2d> initialPoseSupplier;
  private Supplier<Pose2d> poseSupplier;
  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          3.0, 0, 0.0, new Constraints(Units.inchesToMeters(85), Units.inchesToMeters(450.0)));
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          2.5, 0, .01, new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(720.0)));

  private Pose2d targetPose;
  private Pose2d initialPose;
  private double iteration;
  private ArrayList<Pose2d> ControlPoses;
  private double IterationJump;
  private double thetaErrorAbs;
  private double driveErrorAbs;
  private static final double DRIVE_TOLERANCE = 0.005;
  private static final double THETA_TOLERANCE = Units.degreesToRadians(1.0);

  public BezierDriveToPose(
      Drive drive,
      Supplier<Pose2d> targetPoseSupplier,
      Supplier<Pose2d> initialPoseSupplier,
      ArrayList<Pose2d> controlPoses,
      double interationJump,
      double driveTolerance) {
    this.drive = drive;
    this.poseSupplier = targetPoseSupplier;
    addRequirements(drive);
    this.ControlPoses = controlPoses;
    this.IterationJump = interationJump;
    this.initialPoseSupplier = initialPoseSupplier;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    driveController.setTolerance(driveTolerance);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = drive.getPose();
    iteration = 1;
    initialPose = initialPoseSupplier.get();
    targetPose = poseSupplier.get();
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    driveController.setTolerance(DRIVE_TOLERANCE);
    thetaController.setTolerance(THETA_TOLERANCE);
    driveController.reset(currentPose.getTranslation().getDistance(targetPose.getTranslation()));
    thetaController.reset(currentPose.getRotation().getRadians());
  }

  @Override
  public void execute() {
    Pose2d currentPose = drive.getPose();
    double t = (iteration * IterationJump);
    double iterationCheck = 1;
    if (t == 1) {
      end(true);
    }
    double thetaVelocity =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    if (thetaController.atGoal()) thetaVelocity = 0.0;
    Pose2d nextPose =
        new Pose2d(
            Math.pow((1 - t), 2.0) * initialPose.getX()
                + 2 * (1 - t) * t * ControlPoses.get(0).getX()
                + Math.pow(t, 2.0) * targetPose.getX(),
            Math.pow((1 - t), 2.0) * initialPose.getY()
                + 2 * (1 - t) * t * ControlPoses.get(0).getY()
                + Math.pow(t, 2.0) * targetPose.getY(),
            new Rotation2d());
    double currentDistance = currentPose.getTranslation().getDistance(nextPose.getTranslation());
    driveErrorAbs = currentDistance;
    double driveVelocityScalar = driveController.calculate(driveErrorAbs, 0.0);
    if (t == 1) {
      driveVelocityScalar = 0.0;
      iterationCheck = 0;
    }
    Translation2d driveVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(nextPose.getTranslation()).getAngle())
            .transformBy(GeomUtils.transformFromTranslation(driveVelocityScalar, 0.0))
            .getTranslation();
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));
    iteration += iterationCheck;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return driveController.atGoal() && thetaController.atGoal();
  }
}
