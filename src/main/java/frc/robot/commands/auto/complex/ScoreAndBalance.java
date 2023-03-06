package frc.robot.commands.auto.complex;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.holonomictrajectory.Waypoint;
import frc.robot.commands.auto.Balancing;
import frc.robot.commands.auto.Initialize;
import frc.robot.commands.auto.Score;
import frc.robot.commands.drive.GoToTrajectory;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.led.Led2023;
import java.util.List;

public class ScoreAndBalance extends SequentialCommandGroup {

  public ScoreAndBalance(
      String relativePosition,
      String gamePieceType,
      String location,
      Drive drive,
      Arm arm,
      IntakeRelease intakeRelease,
      Led2023 ledStrip) {
    int aprilTag = 7;
    addCommands(
        new Initialize(aprilTag, relativePosition, drive, arm, ledStrip),
        new Score(gamePieceType, location, arm, intakeRelease, ledStrip),
        new GoToTrajectory(
            drive,
            List.of(
                Waypoint.fromDifferentialPose(
                    new Pose2d(
                        new Translation2d(
                            drive.getPose().getTranslation().getX() + Units.inchesToMeters(95.25),
                            drive.getPose().getTranslation().getY()),
                        new Rotation2d())))),
        new Balancing(drive));
  }
}
