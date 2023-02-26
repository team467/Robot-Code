package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.holonomictrajectory.Waypoint;
import frc.robot.FieldConstants;
import frc.robot.commands.arm.ArmScoreMidNodeCMD;
import frc.robot.commands.drive.GoToTrajectory;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import java.util.List;

public class ScoreConeMid extends SequentialCommandGroup {
  public ScoreConeMid(Drive drive, Arm arm) {
    addCommands(
        new GoToTrajectory(
            drive,
            List.of(
                new Waypoint(FieldConstants.aprilTags.get(7).getTranslation().toTranslation2d()))),
        new GoToTrajectory(
            drive,
            List.of(
                new Waypoint(
                    new Translation2d(
                        FieldConstants.aprilTags.get(7).getTranslation().toTranslation2d().getX()
                            + 0.5,
                        FieldConstants.aprilTags.get(7).getTranslation().toTranslation2d().getY()
                            + 0.001)))),
        new ArmScoreMidNodeCMD(arm));
  }
}
