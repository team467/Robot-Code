package frc.robot.commands.auto.better;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.holonomictrajectory.Waypoint;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Community;
import frc.robot.commands.drive.GoToTrajectory;
import frc.robot.subsystems.drive.Drive;
import java.util.List;

public class StraightBack extends SequentialCommandGroup {
  public StraightBack(Drive drive) {
    addCommands(
        new GoToTrajectory(
            drive,
            List.of(
                Waypoint.fromHolonomicPose(
                    FieldConstants.aprilTags
                        .get(8)
                        .toPose2d()
                        .transformBy(
                            new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180)))),
                new Waypoint(
                    new Translation2d(
                        Community.outerX + 1.5, FieldConstants.aprilTags.get(8).getY())))));
  }
}
