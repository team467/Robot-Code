package frc.robot.commands.auto;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.utils.GeomUtils;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.function.Supplier;

public class SplineDriveToPose extends Command {
  private final Drive drive;
  // Supplies the target pose
  private final Supplier<Pose2d> targetPoseSupplier;
  // Supplies the initial pose
  private final Supplier<Pose2d> initialPoseSupplier;
  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          //          2.0, 0, 0.0, new Constraints(Units.inchesToMeters(150),
          // Units.inchesToMeters(450.0)));
          3.0,
          0,
          0.0,
          new Constraints(Units.inchesToMeters(85), Units.inchesToMeters(450.0))); // 90
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController( // TODO: Tune this
          3.0, 0, .01, new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(720.0)));
  // Stores Target Pose
  private Pose2d targetPose;
  // Stores the initial Pose
  private Pose2d initialPose;
  // Waypoints inbetween the target and initial pose to ensure path does not go through solid
  // objects
  private ArrayList<Translation2d> WayPoints;

  private double driveErrorAbs;
  private double thetaErrorAbs;
  // The amount of time the trajectory will take
  private double trajectoryTime;
  // To track the time when the trajectory starts
  private double timeTracker;
  // store trajectory object
  private Trajectory trajectory;
  private static final double DRIVE_TOLERANCE = 0.005;

  private static final double THETA_TOLERANCE = Units.degreesToRadians(1.0);

  public SplineDriveToPose(
      Drive drive,
      Supplier<Pose2d> targetPoseSuppler,
      Supplier<Pose2d> initialPoseSupplier,
      ArrayList<Translation2d> wayPoints,
      double driveTolerance) {
    this.drive = drive;
    this.targetPoseSupplier = targetPoseSuppler;
    this.initialPoseSupplier = initialPoseSupplier;
    this.WayPoints = wayPoints;
    // configure trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(Units.inchesToMeters(85), Units.inchesToMeters(450.0));
    config.setReversed(true);
    // create trajectory
    trajectory =
        TrajectoryGenerator.generateTrajectory(
            initialPoseSupplier.get(), WayPoints, targetPoseSupplier.get(), config);
    addRequirements(drive);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    driveController.setTolerance(DRIVE_TOLERANCE);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = drive.getPose();
    targetPose = targetPoseSupplier.get();
    initialPose = initialPoseSupplier.get();
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    driveController.setTolerance(DRIVE_TOLERANCE);
    thetaController.setTolerance(THETA_TOLERANCE);
    driveController.reset(currentPose.getTranslation().getDistance(targetPose.getTranslation()));
    thetaController.reset(currentPose.getRotation().getRadians());
    trajectoryTime = trajectory.getTotalTimeSeconds();
    // set time tracker to zero since trajectory has not started
    timeTracker = 0;
  }

  @Override
  public void execute() {
    Pose2d currentPose = drive.getPose();
    // gets the next pose in the trajectory based off the time
    var nextPose = trajectory.sample(timeTracker);
    // track trajectory in AdvantageKit
    RobotState.getInstance().pose2d = nextPose.poseMeters;
    double thetaVelocity =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), nextPose.poseMeters.getRotation().getRadians());
    thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(nextPose.poseMeters.getRotation()).getRadians());
    if (thetaController.atGoal()) thetaVelocity = 0.0;

    double currentDistance =
        currentPose.getTranslation().getDistance(nextPose.poseMeters.getTranslation());
    driveErrorAbs = currentDistance;
    double driveVelocityScalar = driveController.calculate(driveErrorAbs, 0.0);
    if (driveController.atGoal()) driveVelocityScalar = 0.0;
    Translation2d driveVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(nextPose.poseMeters.getTranslation()).getAngle())
            .transformBy(GeomUtils.transformFromTranslation(driveVelocityScalar, 0.0))
            .getTranslation();
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));
    timeTracker += 0.02;
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
