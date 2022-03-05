package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;

public abstract class AutonomousModeBase implements AutonomousMode {

  @Override
  public Pose2d getEndPose() {
    Pose2d pose = getStartPose();
    for (Trajectory trajectory : getTrajectories()) {
      List<State> states = trajectory.relativeTo(pose).getStates();
      pose = states.get(states.size() - 1).poseMeters;
    }

    return pose;
  }

  @Override
  public void addFieldInformation(Field2d field2d) {
    field2d.setRobotPose(getStartPose());
  }
}
