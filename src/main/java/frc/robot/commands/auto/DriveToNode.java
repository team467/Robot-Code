package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.utils.AllianceFlipUtil;
import frc.robot.FieldConstants.Grids;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class DriveToNode extends QuickDriveToPose {

  public DriveToNode(Drive drive, Supplier<Integer> row) {
    super(
        drive,
        AllianceFlipUtil.apply(
            new Pose2d(
                new Translation2d(
                    Grids.lowX,
                    Grids.nodeFirstY + Grids.nodeSeparationY * row.get()), // TODO: tune x
                new Rotation2d())));
  }
}
