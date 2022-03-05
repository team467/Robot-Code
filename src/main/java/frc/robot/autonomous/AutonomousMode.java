package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;

public interface AutonomousMode {
  String getName();
  Command getCommand();
  Pose2d getStartPose(StartingConfiguration startingConfiguration);
  Pose2d getEndPose(StartingConfiguration startingConfiguration);
  Trajectory[] getTrajectories(StartingConfiguration startingConfiguration);
  void addFieldInformation(Field2d field2d);
}
