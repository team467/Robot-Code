package frc.robot.commands.auto;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.Drive;
import java.util.List;
import java.util.function.Supplier;

public class PathOnTheFly {
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

  public PathPlannerPath createPath() {
    currentPose = drive::getPose;
    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(currentPose.get(), targetPose.get());
    PathPlannerPath path =
        new PathPlannerPath(
            waypoints, constraints, null, new GoalEndState(0, targetPose.get().getRotation()));
    path.preventFlipping = preventFlipping;
    return path;
  }
}
