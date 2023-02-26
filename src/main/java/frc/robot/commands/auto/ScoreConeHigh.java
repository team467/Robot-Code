package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.lib.holonomictrajectory.Waypoint;
import frc.robot.FieldConstants;
import frc.robot.commands.arm.ArmScoreHighNodeCMD;
import frc.robot.commands.drive.GoToTrajectory;

public class ScoreConeHigh extends SequentialCommandGroup 
{
  public ScoreConeHigh(Drive drive, Arm arm) 
  {
    addCommands(new GoToTrajectory(drive,
                List.of(
                new Waypoint(
                FieldConstants.aprilTags.get(7).getTranslation().toTranslation2d()))),
                  new GoToTrajectory(drive,
        List.of(
          new Waypoint(
            new Translation2d(
                FieldConstants.aprilTags.get(7).getTranslation().toTranslation2d().getX() + 0.5,
                          FieldConstants.aprilTags.get(7).getTranslation().toTranslation2d().getY())))),
                  new ArmScoreHighNodeCMD(arm));
  }  
}
