package frc.robot.commands.auto.better;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.holonomictrajectory.Waypoint;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Community;
import frc.robot.commands.drive.GoToTrajectory;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.Led2023;
import java.util.List;

public class LeaveStraight8 extends SequentialCommandGroup {
  public LeaveStraight8(Drive drive, Arm arm, Led2023 led2023) {
    arm.setCalibratedAssumeHomePosition();
    led2023.setArmCalibrated();
    // addCommands(Commands.runOnce(() ->
    // drive.setPose(FieldConstants.aprilTags.get(8).toPose2d())));
    addCommands(
        Commands.runOnce(() -> drive.setPose(FieldConstants.aprilTags.get(8).toPose2d())),
        new GoToTrajectory(
            drive,
            List.of(
                Waypoint.fromHolonomicPose(
                    FieldConstants.aprilTags
                        .get(8)
                        .toPose2d()
                        .transformBy(
                            new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180)))),
                Waypoint.fromDifferentialPose(
                    new Pose2d(
                        new Translation2d(
                            Community.outerX + 1.5, FieldConstants.aprilTags.get(8).getY()),
                        new Rotation2d())))));
  }
}
