package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.utils.AllianceFlipUtil;
import frc.robot.FieldConstants.Grids;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.intakerelease.IntakeRelease.Wants;
import java.util.function.Supplier;

public class NewAlignToNode extends CommandBase {
  private final Drive drive;
  private final Supplier<Wants> wants;
  private Pose2d alignPose;
  private StraightDriveToPose alignCommand;

  public NewAlignToNode(Drive drive, IntakeRelease intakeRelease) {
    this.drive = drive;
    this.wants = intakeRelease::getWants;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    double robotY = drive.getPose().getY();
    double closestNodeY;
    if (wants.get() == Wants.CONE) {
      // 0,2,3,5,6,8
      closestNodeY = Grids.nodeY[0];
      for (int i = 0; i < Grids.nodeY.length; i++) {
        if (i == 1 || i == 4 || i == 7) {
          continue;
        }
        if (Math.abs(robotY - Grids.nodeY[i]) < Math.abs(robotY - closestNodeY)) {
          closestNodeY = Grids.nodeY[i];
        }
      }
    } else {
      // 1,4,7
      closestNodeY = Grids.nodeY[1];
      for (int i = 1; i < Grids.nodeY.length; i += 3) {
        if (Math.abs(robotY - Grids.nodeY[i]) < Math.abs(robotY - closestNodeY)) {
          closestNodeY = Grids.nodeY[i];
        }
      }
    }
    alignPose =
        AllianceFlipUtil.apply(
            new Pose2d(
                Grids.outerX + Units.inchesToMeters(19), closestNodeY, new Rotation2d(Math.PI)));
    alignCommand = new StraightDriveToPose(alignPose, drive);
    alignCommand.initialize();
  }

  @Override
  public void execute() {
    alignCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    alignCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return alignCommand.isFinished();
  }
}
