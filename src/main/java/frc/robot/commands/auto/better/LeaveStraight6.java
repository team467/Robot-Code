package frc.robot.commands.auto.better;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.holonomictrajectory.Waypoint;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Community;
import frc.robot.commands.arm.ArmCalibrateCMD;
import frc.robot.commands.drive.GoToTrajectory;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import java.util.List;

public class LeaveStraight6 extends SequentialCommandGroup {
  public LeaveStraight6(Drive drive, Arm arm) {
    addCommands(new ArmCalibrateCMD(arm));
    addCommands(
        new GoToTrajectory(
            drive,
            List.of(
                Waypoint.fromHolonomicPose(
                    FieldConstants.aprilTags
                        .get(6)
                        .toPose2d()
                        .transformBy(
                            new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180)))),
                Waypoint.fromDifferentialPose(
                    new Pose2d(
                        new Translation2d(Community.midX, FieldConstants.aprilTags.get(6).getY()),
                        new Rotation2d())))));
  }
}
