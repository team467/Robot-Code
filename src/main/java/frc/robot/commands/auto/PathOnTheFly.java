package frc.robot.commands.auto;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.List;
import java.util.function.Supplier;

public class PathOnTheFly extends Command {
  private Supplier<Pose2d> currentPose;
  private Supplier<Pose2d> targetPose;
  private final boolean preventFlipping = true;
  private Drive drive;
  private double driveTolerance;
  private double thetaTolerance;
  private final PathConstraints constraints =
      new PathConstraints(3.5, 12.7, Units.degreesToRadians(540), Units.degreesToRadians(720));

  public PathOnTheFly(Drive drive, Supplier<Pose2d> currentPose, Supplier<Pose2d> targetPose) {
    this.drive = drive;
    this.currentPose = currentPose;
    this.targetPose = targetPose;
    this.driveTolerance = 0.004;
    this.thetaTolerance = Units.degreesToRadians(1.0);
  }

  public PathOnTheFly(Drive drive, Supplier<Pose2d> targetPose) {
    this.drive = drive;
    this.currentPose = drive::getPose;
    this.targetPose = targetPose;
    this.driveTolerance = 0.004;
    this.thetaTolerance = Units.degreesToRadians(1.0);
  }

  public PathOnTheFly(
      Drive drive, Supplier<Pose2d> targetPose, double driveTolerance, double thetaTolerance) {
    this.drive = drive;
    this.currentPose = drive::getPose;
    this.targetPose = targetPose;
    this.driveTolerance = driveTolerance;
    this.thetaTolerance = thetaTolerance;
  }

  @Override
  public void initialize() {
    currentPose = drive::getPose;
    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(currentPose.get(), targetPose.get());
    PathPlannerPath path =
        new PathPlannerPath(
            waypoints, constraints, null, new GoalEndState(0, targetPose.get().getRotation()));
    path.preventFlipping = preventFlipping;
    drive.runPath(path);
  }

  @Override
  public void execute() {
    var currentPose = drive.getPose();
    double deltaDistance =
        currentPose.getTranslation().getDistance(targetPose.get().getTranslation());
    double deltaTheta =
        Math.abs(
            currentPose.getRotation().getDegrees() - targetPose.get().getRotation().getDegrees());
    if (deltaDistance < driveTolerance) {
      end(true);
    }
  }

  public void end(boolean interrupted) {
    drive.stop();
  }
}
