package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.holonomictrajectory.Waypoint;
import frc.robot.FieldConstants;
import frc.robot.commands.drive.GoToTrajectory;
import frc.robot.subsystems.drive.Drive;

public class CrossCommunity extends SequentialCommandGroup {
  /** Creates a new DrivelessScore. */
  public CrossCommunity() {
    // Use addRequirements() here to declare subsystem dependencies.
    Drive drive = new Drive(null, null, null, null, null);
    addCommands(
    //commandA,
    //commandB
    new GoToTrajectory(drive, List.of(Waypoint.fromHolonomicPose(new Pose2d()),
        new Waypoint(new Translation2d(FieldConstants.Community.outerX, 0)))));
  }
}
