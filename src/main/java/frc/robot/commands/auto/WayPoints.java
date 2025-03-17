package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class WayPoints {
  public final Pose2d startPose;

  public WayPoints(Pose2d startPose) {
    this.startPose = startPose;
  }

  public Pose2d getPose() {
    return startPose;
  }

  public Rotation2d getRotation() {
    return startPose.getRotation();
  }
}
