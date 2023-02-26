package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.holonomictrajectory.Waypoint;
import frc.robot.FieldConstants;
import frc.robot.commands.arm.ArmCalibrateCMD;
import frc.robot.commands.arm.ArmScoreMidNodeCMD;
import frc.robot.commands.drive.GoToTrajectory;
import frc.robot.commands.intakerelease.WantConeCMD;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.led.Led2023;
import java.util.List;

public class ScoreConeMid extends SequentialCommandGroup {
  public ScoreConeMid(Drive drive, Arm arm, IntakeRelease intakeRelease, Led2023 ledStrip) {
    addCommands(
        new ArmCalibrateCMD(arm),
        new WantConeCMD(intakeRelease, ledStrip),
        new GoToTrajectory(
            drive,
            List.of(
                new Waypoint(FieldConstants.aprilTags.get(7).getTranslation().toTranslation2d()),
                new Waypoint(
                    new Translation2d(
                        FieldConstants.aprilTags.get(7).getTranslation().toTranslation2d().getX()
                            + 0.3,
                        FieldConstants.aprilTags.get(7).getTranslation().toTranslation2d().getY()
                            + FieldConstants.Grids.nodeSeparationY)))),
        new ArmScoreMidNodeCMD(arm, intakeRelease));
  }
}
