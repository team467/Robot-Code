package frc.robot.commands.auto.complex;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.Initialize;
import frc.robot.commands.auto.Score;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.led.Led2023;

public class ScoreAndStop extends SequentialCommandGroup {
  public ScoreAndStop(
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
        new Score(gamePieceType, location, arm, intakeRelease, ledStrip));
  }
}
