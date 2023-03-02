package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.holonomictrajectory.Waypoint;
import frc.robot.FieldConstants;
import frc.robot.commands.arm.ArmCalibrateCMD;
import frc.robot.commands.arm.ArmScoreHighNodeCMD;
import frc.robot.commands.drive.GoToTrajectory;
import frc.robot.commands.intakerelease.ReleaseCMD;
import frc.robot.commands.intakerelease.WantCubeCMD;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.led.Led2023;
import java.util.List;

public class ScoreThenMoveOut8 extends SequentialCommandGroup {
  public ScoreThenMoveOut8(Drive drive, Arm arm, IntakeRelease intakeRelease, Led2023 ledStrip) {
    addCommands(
        new ArmCalibrateCMD(arm),
        new WantCubeCMD(intakeRelease, ledStrip),
        new ArmScoreHighNodeCMD(arm, intakeRelease),
        new GoToTrajectory(
            drive,
            List.of(
                new Waypoint(
                    new Translation2d(
                        FieldConstants.Community.outerX,
                        FieldConstants.aprilTags
                            .get(8)
                            .getTranslation()
                            .toTranslation2d()
                            .getY())))));
    new ReleaseCMD(intakeRelease, ledStrip, arm);
  }
}
