package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.utils.AllianceFlipUtil;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Grids;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class AlignToNode extends StraightDriveToPose {

  public AlignToNode(Drive drive, Supplier<Integer> row) {
    super(
        AllianceFlipUtil.apply(
            new Pose2d(
                new Translation2d(
                    FieldConstants.aprilTags.getTagPose(7).get().getTranslation().getX()
                        + Units.inchesToMeters(16),
                    Grids.nodeFirstY + Grids.nodeSeparationY * 5), // TODO: tune x
                new Rotation2d(Math.PI))),
        drive);
  }
}
