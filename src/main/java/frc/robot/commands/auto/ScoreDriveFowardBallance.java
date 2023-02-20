package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;

public class ScoreDriveFowardBallance extends SequentialCommandGroup {
  /** Creates a new DrivelessScore. */
  public ScoreDriveFowardBallance(Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
        // enter driveless score here
        new DriveFowardBallance(drive));
  }
}
