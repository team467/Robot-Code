package frc.robot.commands.auto;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class TrajectoryFollower extends Command {
  private final Drive drive;
  private final Supplier<Pose2d> targetPoseSupplier;
  //  private Pose2d targetPose;
  //  private final TrajectoryConfig trajConfig;
  //  private final Trajectory traj;

  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          3.0, 0, 0.1, new Constraints(Units.inchesToMeters(125), Units.inchesToMeters(500)));
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          2.5, 0, .01, new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(720.0)));
  private static final double DRIVE_TOLERANCE = 0.005;
  private static final double THETA_TOLERANCE = Units.degreesToRadians(1.0);

  public TrajectoryFollower(Drive drive, Supplier<Pose2d> targetPoseSupplier) {
    this.drive = drive;
    this.targetPoseSupplier = targetPoseSupplier;
  }

  @Override
  public void initialize() {
    //    Pose2d currentPose = drive.getPose();
    //    targetPose = targetPoseSupplier.get();
    //    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    //    driveController.setTolerance(DRIVE_TOLERANCE);
    //    thetaController.setTolerance(THETA_TOLERANCE);
    ////    trajConfig.setStartVelocity(drive.getVelocityInMetersPerSec());
    //    trajConfig.setEndVelocity(0.0);
    //    trajConfig.setReversed(true);
  }

  @Override
  public void execute() {
    Pose2d currentPose = drive.getPose();
  }
}
