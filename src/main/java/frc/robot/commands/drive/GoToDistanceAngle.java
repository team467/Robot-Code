package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.holonomictrajectory.Waypoint;
import frc.lib.utils.GeomUtils;
import frc.robot.subsystems.drive.Drive;
import java.util.List;

/**
 * TODO: do not use, BROKEN! BROKEN! BROKEN! BROKEN! BROKEN! BROKEN!
 *
 * @deprecated BROKEN! BROKEN! BROKEN!
 */
@Deprecated
public class GoToDistanceAngle extends CommandBase {
  private GoToTrajectory command;
  private boolean finished = false;

  public GoToDistanceAngle(Drive drive, double distance, Rotation2d angle) {
    if (distance == 0) {
      finished = true;
    } else {
      command =
          new GoToTrajectory(
              drive,
              List.of(
                  Waypoint.fromHolonomicPose(drive.getPose()),
                  Waypoint.fromDifferentialPose(
                      drive
                          .getPose()
                          .plus(
                              GeomUtils.transformFromTranslation(
                                  new Translation2d(distance, angle))))));
    }

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    command.initialize();
  }

  @Override
  public void execute() {
    command.execute();
  }

  @Override
  public void end(boolean interrupted) {
    command.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return command.isFinished() || finished;
  }
}
