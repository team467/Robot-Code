package frc.robot.commands.auto.better;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.holonomictrajectory.Waypoint;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Community;
import frc.robot.commands.arm.ArmCalibrateZeroAtHomeCMD;
import frc.robot.commands.drive.GoToTrajectory;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.Led2023;
import java.util.List;

public class LeaveStraight8Balance extends SequentialCommandGroup {
  public LeaveStraight8Balance(Drive drive, Arm arm, Led2023 led2023) {
    Pose2d startingPosition =
        new Pose2d(
            new Translation2d(
                Community.outerX + 1.5,
                (Community.chargingStationLeftY + Community.chargingStationRightY) / 2),
            new Rotation2d(Math.PI));
    boolean enterFront =
        startingPosition.getX()
            < (Community.chargingStationInnerX + Community.chargingStationOuterX) / 2.0;
    Pose2d position0 =
        new Pose2d(
            enterFront ? Community.chargingStationInnerX : Community.chargingStationOuterX,
            //            enterFront
            //                ? Community.chargingStationInnerX - 0.6
            //                : Community.chargingStationOuterX + 0.6,
            MathUtil.clamp(
                startingPosition.getY(),
                Community.chargingStationRightY + 0.6,
                Community.chargingStationLeftY - 0.6),
            Rotation2d.fromDegrees(startingPosition.getRotation().getCos() > 0.0 ? 0.0 : 180.0));
    Pose2d position1 =
        new Pose2d(
            (Community.chargingStationOuterX + Community.chargingStationInnerX) / 2.0,
            position0.getY(),
            position0.getRotation());

    addCommands(new ArmCalibrateZeroAtHomeCMD(arm, led2023));
    addCommands(
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
                        new Translation2d(Community.outerX, FieldConstants.aprilTags.get(6).getY()),
                        new Rotation2d())))));
  }
}
