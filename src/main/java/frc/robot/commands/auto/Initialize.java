package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.Led2023;

public class Initialize extends SequentialCommandGroup {
  public Initialize(int aprilTag, String relativePosition, Drive drive, Arm arm, Led2023 ledStrip) {
    Translation2d aprilTagLocation = FieldConstants.aprilTags.get(aprilTag).getTranslation().toTranslation2d();
    double relativePositionOffset = 0;
    if (relativePosition.equals("Left")) {
      relativePositionOffset = - 22;
    } else if (relativePosition.equals("Right")) {
      relativePositionOffset = 22;
    }
    Pose2d initialPose = new Pose2d(new Translation2d(
        aprilTagLocation.getX() + Units.inchesToMeters(16),
        aprilTagLocation.getY() + Units.inchesToMeters(relativePositionOffset)),
        Rotation2d.fromDegrees(180));
    addCommands(
      Commands.runOnce(() -> arm.setCalibratedAssumeHomePosition()),
      Commands.runOnce(() -> ledStrip.setArmCalibrated()),
      Commands.runOnce(() -> drive.setPose(initialPose))
    );
  }
}
