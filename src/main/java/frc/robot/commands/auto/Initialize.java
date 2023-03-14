package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.lib.utils.AllianceFlipUtil;
import frc.robot.FieldConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class Initialize extends ParallelCommandGroup {
  public Initialize(int aprilTag, String relativePosition, Drive drive, Arm arm) {
    Translation2d aprilTagLocation =
        FieldConstants.aprilTags.getTagPose(aprilTag).get().getTranslation().toTranslation2d();
    final double relativePositionOffset;
    if (relativePosition.equals("Left")) {
      relativePositionOffset = -22;
    } else if (relativePosition.equals("Right")) {
      relativePositionOffset = 22;
    } else {
      relativePositionOffset = 0;
    }

    double distanceAprilTagToEdgeOfNode = 16;
    double distanceRobotFrontToCenter = 12.75;
    Supplier<Pose2d> expectedPose =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(
                    new Translation2d(
                        aprilTagLocation.getX()
                            + Units.inchesToMeters(
                                distanceAprilTagToEdgeOfNode + distanceRobotFrontToCenter),
                        aprilTagLocation.getY() + Units.inchesToMeters(relativePositionOffset)),
                    Rotation2d.fromDegrees(180)));

    addCommands(
        Commands.runOnce(arm::setCalibratedAssumeHomePosition),
        Commands.runOnce(
            () -> {
              Pose2d measuredPose = drive.getPose();
              if (measuredPose == null
                  || (measuredPose.getX() < 0.1 && measuredPose.getY() <= 0.0)
                  || measuredPose.getTranslation().getDistance(expectedPose.get().getTranslation())
                      > Units.inchesToMeters(18.0)) {
                drive.setPose(expectedPose.get());
                DriverStation.reportWarning(
                    "WARNING: Robot pose is not accurate. \n"
                        + "Expected pose: "
                        + expectedPose
                        + "\n"
                        + "Measured pose: "
                        + measuredPose,
                    false);
              }
            }));
  }
}
