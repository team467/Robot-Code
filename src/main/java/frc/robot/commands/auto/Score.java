package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmScoreHighNodeCMD;
import frc.robot.commands.arm.ArmScoreLowNodeCMD;
import frc.robot.commands.arm.ArmScoreMidNodeCMD;
import frc.robot.commands.intakerelease.IntakeCMD;
import frc.robot.commands.intakerelease.ReleaseCMD;
import frc.robot.commands.intakerelease.WantConeCMD;
import frc.robot.commands.intakerelease.WantCubeCMD;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.intakerelease.IntakeRelease.Wants;

public class Score extends SequentialCommandGroup {

  public Score(String gamePieceType, String location, Arm arm, IntakeRelease intakeRelease) {
    add(gamePieceType, location, arm, intakeRelease);
  }

  public Score(String location, Arm arm, IntakeRelease intakeRelease) {
    if (intakeRelease.getWants() == Wants.CONE) {
      add("Cone", location, arm, intakeRelease);
    } else {
      add("Cube", location, arm, intakeRelease);
    }
  }

  private void add(String gamePieceType, String location, Arm arm, IntakeRelease intakeRelease) {
    addCommands(
        pieceType(gamePieceType, intakeRelease),
        Commands.parallel(
            new IntakeCMD(intakeRelease), armLocationCommand(location, arm, intakeRelease)),
        new ReleaseCMD(intakeRelease, arm));
  }

  private Command armLocationCommand(String location, Arm arm, IntakeRelease intakeRelease) {
    if (location.equalsIgnoreCase("high")) {
      return new ArmScoreHighNodeCMD(arm, intakeRelease);
    } else if (location.equalsIgnoreCase("mid")) {
      return new ArmScoreMidNodeCMD(arm, intakeRelease);
    } else {
      return new ArmScoreLowNodeCMD(arm, intakeRelease);
    }
  }

  private Command pieceType(String gamePiece, IntakeRelease intakeRelease) {
    if (gamePiece.equalsIgnoreCase("cone")) {
      return new WantConeCMD(intakeRelease);
    } else {
      return new WantCubeCMD(intakeRelease);
    }
  }
}
