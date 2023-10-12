package frc.robot.commands.auto.complex;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
// import frc.robot.commands.arm.ArmHomeCMD;
import frc.robot.commands.auto.BetterBalancing;
import frc.robot.commands.auto.Initialize;
import frc.robot.commands.auto.Score;
import frc.robot.commands.auto.StraightDriveToPose;
// import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intakerelease.IntakeRelease;

public class ScoreAndBackUpAndBalance extends SequentialCommandGroup {
  public ScoreAndBackUpAndBalance(
      String relativePosition,
      String gamePieceType,
      String location,
      Drive drive,
      // Arm arm,
      IntakeRelease intakeRelease) {
    int aprilTag = 7;
    addCommands(
        new Initialize(aprilTag, relativePosition, drive
            // , arm
         ),
        new Score(gamePieceType, location,
        //  arm,
          intakeRelease),
        Commands.parallel(
            // new ArmHomeCMD(arm, intakeRelease::wantsCone).withTimeout(3.5),
            Commands.sequence(
                new StraightDriveToPose(0.0, -FieldConstants.Grids.nodeSeparationY, 0.0, drive),
                new StraightDriveToPose(Units.inchesToMeters(175.0), 0.0, 0.0, drive))),
        new StraightDriveToPose(Units.inchesToMeters(-75.0), 0.0, 0.0, drive).withTimeout(2.5),
        new BetterBalancing(drive));
  }
}
