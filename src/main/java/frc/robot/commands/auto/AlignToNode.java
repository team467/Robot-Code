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

public class AlignToNode extends QuickDriveToPose {

  public AlignToNode(Drive drive, Supplier<Integer> row) {
    super(
        drive,
        AllianceFlipUtil.apply(
            new Pose2d(
                new Translation2d(
                    FieldConstants.aprilTags.getTagPose(7).get().getX()
                        + Units.inchesToMeters(12.0 + 4.0),
                    Grids.nodeY[5]), // TODO: tune x
                new Rotation2d(Math.PI))));
  }
}
