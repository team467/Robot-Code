package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.holonomictrajectory.Waypoint;
import frc.robot.FieldConstants;
import frc.robot.commands.drive.GoToTrajectory;
import frc.robot.subsystems.drive.Drive;
import java.util.List;

public class DriveFowardBallance extends SequentialCommandGroup {
  /** Creates a new DrivelessScore. */
  public DriveFowardBallance(Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
        new GoToTrajectory(
            drive,
            List.of(
                new Waypoint(FieldConstants.aprilTags.get(7).getTranslation().toTranslation2d()),
                new Waypoint(
                    new Translation2d(
                        FieldConstants.Community.outerX + 0.5,
                        FieldConstants.aprilTags
                            .get(7)
                            .getTranslation()
                            .toTranslation2d()
                            .getY())))),
        new GoToTrajectory(
            drive,
            List.of(
                new Waypoint(
                    new Translation2d(
                        FieldConstants.Community.outerX + 2.0,
                        FieldConstants.aprilTags.get(7).getTranslation().toTranslation2d().getY())),
                new Waypoint(
                    new Translation2d(
                        FieldConstants.Community.chargingStationOuterX,
                        FieldConstants.aprilTags
                            .get(7)
                            .getTranslation()
                            .toTranslation2d()
                            .getY())))),
        new Balancing(drive));
  }
}
