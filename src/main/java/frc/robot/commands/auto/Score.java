package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmScoreHighNodeCMD;
import frc.robot.commands.arm.ArmScoreLowNodeCMD;
import frc.robot.commands.arm.ArmScoreMidNodeCMD;
import frc.robot.commands.effector.IntakeCMD;
import frc.robot.commands.effector.ReleaseCMD;
import frc.robot.commands.effector.WantConeCMD;
import frc.robot.commands.effector.WantCubeCMD;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.effector.Effector;
import frc.robot.subsystems.effector.Effector.Wants;

public class Score extends SequentialCommandGroup {

  public Score(String gamePieceType, String location, Arm arm, Effector effector) {
    add(gamePieceType, location, arm, effector);
  }

  public Score(String location, Arm arm, Effector effector) {
    if (effector.getWants() == Wants.CONE) {
      add("Cone", location, arm, effector);
    } else {
      add("Cube", location, arm, effector);
    }
  }

  private void add(String gamePieceType, String location, Arm arm, Effector effector) {
    addCommands(
        pieceType(gamePieceType, effector),
        Commands.parallel(new IntakeCMD(effector), armLocationCommand(location, arm, effector)),
        new ReleaseCMD(effector, arm));
  }

  private Command armLocationCommand(String location, Arm arm, Effector effector) {
    if (location.equalsIgnoreCase("high")) {
      return new ArmScoreHighNodeCMD(arm, effector::wantsCone);
    } else if (location.equalsIgnoreCase("mid")) {
      return new ArmScoreMidNodeCMD(arm, effector::wantsCone);
    } else {
      return new ArmScoreLowNodeCMD(arm);
    }
  }

  private Command pieceType(String gamePiece, Effector effector) {
    if (gamePiece.equalsIgnoreCase("cone")) {
      return new WantConeCMD(effector);
    } else {
      return new WantCubeCMD(effector);
    }
  }
}
