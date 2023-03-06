package frc.robot.commands.auto.complex;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.holonomictrajectory.Waypoint;
import frc.robot.commands.auto.BackUp;
import frc.robot.commands.auto.Balancing;
import frc.robot.commands.auto.Initialize;
import frc.robot.commands.auto.Score;
import frc.robot.commands.drive.GoToTrajectory;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.led.Led2023;
import java.util.List;

public class ScoreAndBackUp extends SequentialCommandGroup {

  public ScoreAndBackUp(
      int aprilTag,
      String relativePosition,
      String gamePieceType,
      String location,
      Drive drive,
      Arm arm,
      IntakeRelease intakeRelease,
      Led2023 ledStrip) {
    addCommands(
        new Initialize(aprilTag, relativePosition, drive, arm, ledStrip),
        new Score(gamePieceType, location, arm, intakeRelease, ledStrip),
        new BackUp(drive, arm, ledStrip));
    if (aprilTag == 7) {
      balance(drive);
    }
  }

  private void balance(Drive drive) {
    andThen(
        new GoToTrajectory(
            drive,
            List.of(
                Waypoint.fromDifferentialPose(
                    new Pose2d(
                        new Translation2d(
                            drive.getPose().getTranslation().getX() + Units.inchesToMeters(-55),
                            drive.getPose().getTranslation().getY()),
                        new Rotation2d())))));
    andThen(new Balancing(drive));
  }
}
