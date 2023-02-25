package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.holonomictrajectory.Waypoint;
import frc.robot.FieldConstants;
import frc.robot.commands.drive.GoToTrajectory;
import frc.robot.subsystems.drive.Drive;
import java.util.List;

public class Tag8Balance extends SequentialCommandGroup {
  public Tag8Balance(Drive drive) {
    addCommands(
        new GoToTrajectory(
            drive,
            List.of(
                new Waypoint(FieldConstants.aprilTags.get(8).getTranslation().toTranslation2d()),
                new Waypoint(
                    new Translation2d(
                        FieldConstants.Community.outerX + 1,
                        FieldConstants.aprilTags
                            .get(8)
                            .getTranslation()
                            .toTranslation2d()
                            .getY()))),
            0,
            0,
            List.of(new MaxVelocityConstraint(0.4))),
        new GoToTrajectory(
            drive,
            List.of(
                new Waypoint(
                    new Translation2d(
                        FieldConstants.Community.outerX + 1,
                        FieldConstants.aprilTags.get(7).getTranslation().toTranslation2d().getY())),
                new Waypoint(
                    new Translation2d(
                        FieldConstants.Community.chargingStationOuterX,
                        FieldConstants.aprilTags
                            .get(7)
                            .getTranslation()
                            .toTranslation2d()
                            .getY()))),
            0,
            0,
            List.of(new MaxVelocityConstraint(0.4))),
        new Balancing(drive));
  }
}
