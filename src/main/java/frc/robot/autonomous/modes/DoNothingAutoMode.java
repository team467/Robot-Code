package frc.robot.autonomous.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.autonomous.AutonomousModeBase;
import frc.robot.autonomous.StartingConfiguration;
import frc.robot.vision.HubTarget;

public class DoNothingAutoMode extends AutonomousModeBase {
  @Override
  public String getName() {
    return "Do Nothing";
  }

  @Override
  public Command getCommand() {
    return new RunCommand(() -> {});
  }

  @Override
  public Pose2d getStartPose(StartingConfiguration startingConfiguration) {
    return startingConfiguration.getDefaultPose();
  }

  @Override
  public Pose2d getEndPose(StartingConfiguration startingConfiguration) {
    return startingConfiguration.getDefaultPose();
  }

  @Override
  public Trajectory[] getTrajectories(StartingConfiguration startingConfiguration) {
    return new Trajectory[0];
  }

}
