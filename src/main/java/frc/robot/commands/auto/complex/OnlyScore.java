package frc.robot.commands.auto.complex;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmHomeCMD;
import frc.robot.commands.auto.Initialize;
import frc.robot.commands.auto.Score;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.effector.Effector;

public class OnlyScore extends SequentialCommandGroup {
  public OnlyScore(
      int aprilTag,
      String relativePosition,
      String gamePieceType,
      String location,
      Drive drive,
      Arm arm,
      Effector effector) {
    addCommands(
        new Initialize(aprilTag, relativePosition, drive, arm),
        new Score(gamePieceType, location, arm, effector),
        new ArmHomeCMD(arm, effector::wantsCone));
  }
}
