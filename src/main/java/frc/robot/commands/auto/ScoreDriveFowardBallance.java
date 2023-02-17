package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.holonomictrajectory.Waypoint;
import frc.robot.FieldConstants;
import frc.robot.commands.drive.Balancing;
import frc.robot.commands.drive.GoToTrajectory;
import frc.robot.subsystems.drive.Drive;
import java.util.List;

public class ScoreDriveFowardBallance extends SequentialCommandGroup {
  /** Creates a new DrivelessScore. */
  public ScoreDriveFowardBallance(Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
        // commandA,
        // commandB

        // enter driveless score here

        // below is drivefowardballance
        new GoToTrajectory(
            drive,
            List.of(
                new Waypoint(FieldConstants.aprilTags.get(7).getTranslation().toTranslation2d()),
                new Waypoint(
                    new Translation2d(
                        FieldConstants.Community.outerX + 0.5,
                        FieldConstants.aprilTags.get(7).getTranslation().toTranslation2d().getY())),
                new Waypoint(
                    new Translation2d(
                        FieldConstants.Community.outerX,
                        FieldConstants.aprilTags
                            .get(7)
                            .getTranslation()
                            .toTranslation2d()
                            .getY())))),
        new Balancing(drive));
  }
}
