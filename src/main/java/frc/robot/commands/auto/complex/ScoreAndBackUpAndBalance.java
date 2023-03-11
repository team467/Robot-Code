package frc.robot.commands.auto.complex;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmHomeCMD;
import frc.robot.commands.auto.BetterBalancing;
import frc.robot.commands.auto.Initialize;
import frc.robot.commands.auto.Score;
import frc.robot.commands.auto.StraightDriveToPose;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.led.Led2023;

public class ScoreAndBackUpAndBalance extends SequentialCommandGroup {
  public ScoreAndBackUpAndBalance(
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
        Commands.parallel(
            new StraightDriveToPose(Units.inchesToMeters(170.0), 0.0, 0.0, drive),
            new ArmHomeCMD(arm, ledStrip)),
        new StraightDriveToPose(Units.inchesToMeters(-75.0), 0.0, 0.0, drive),
        new BetterBalancing(drive));
  }
}
