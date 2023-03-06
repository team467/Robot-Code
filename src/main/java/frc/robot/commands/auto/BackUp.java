package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.lib.holonomictrajectory.Waypoint;
import frc.robot.commands.arm.ArmHomeCMD;
import frc.robot.commands.drive.GoToTrajectory;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.Led2023;
import java.util.List;

public class BackUp extends ParallelCommandGroup {
  public BackUp(Drive drive, Arm arm, Led2023 ledStrip) {
    addCommands(
        new GoToTrajectory(
            drive,
            List.of(
                Waypoint.fromDifferentialPose(
                    new Pose2d(
                        new Translation2d(
                            drive.getPose().getTranslation().getX() + Units.inchesToMeters(150),
                            drive.getPose().getTranslation().getY()),
                        new Rotation2d())))),
        new ArmHomeCMD(arm, ledStrip));
  }
}
