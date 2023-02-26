package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.lib.holonomictrajectory.Waypoint;
import frc.robot.FieldConstants;
import frc.robot.commands.drive.GoToTrajectory;
import frc.robot.subsystems.arm.Arm;

public class ScoreDriveFowardBallance extends SequentialCommandGroup {
  /** Creates a new DrivelessScore. */
  public ScoreDriveFowardBallance(Drive drive, Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new ScoreConeHigh(drive, arm),
        new GoToTrajectory(drive,
        List.of(
          new Waypoint(
            new Translation2d(
                FieldConstants.Community.innerX + 0.5,
                          FieldConstants.aprilTags.get(7).getTranslation().toTranslation2d().getY())))),
                          new Balancing(drive));
        // add in driveless score

  }
}
