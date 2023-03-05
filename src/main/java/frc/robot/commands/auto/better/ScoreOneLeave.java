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
import frc.robot.commands.arm.ArmScoreHighNodeCMD;
import frc.robot.commands.drive.GoToTrajectory;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.led.Led2023;
import java.util.List;

public class ScoreOneLeave extends SequentialCommandGroup {
  public ScoreOneLeave(Drive drive, Arm arm, IntakeRelease intakeRelease, Led2023 ledStrip) {
    arm.setCalibratedAssumeHomePosition();
    ledStrip.setArmCalibrated();
    // addCommands(new ArmCalibrateZeroAtHomeCMD(arm, ledStrip));
    addCommands(Commands.runOnce(() -> drive.setPose(FieldConstants.aprilTags.get(6).toPose2d())));
    addCommands(new ArmScoreHighNodeCMD(arm, intakeRelease, ledStrip));
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
                        new Translation2d(
                            Community.midX, (Community.chargingStationLeftY + Community.leftY) / 2),
                        new Rotation2d())))));
  }
}
