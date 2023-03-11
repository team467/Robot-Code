package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmScoreHighNodeCMD;
import frc.robot.commands.arm.ArmScoreLowNodeCMD;
import frc.robot.commands.arm.ArmScoreMidNodeCMD;
import frc.robot.commands.intakerelease.ReleaseCMD;
import frc.robot.commands.intakerelease.WantConeCMD;
import frc.robot.commands.intakerelease.WantCubeCMD;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.intakerelease.IntakeRelease.Wants;
import frc.robot.subsystems.led.Led2023;

public class Score extends SequentialCommandGroup {

  public Score(
      String gamePieceType,
      String location,
      Arm arm,
      IntakeRelease intakeRelease,
      Led2023 ledStrip) {
    add(gamePieceType, location, arm, intakeRelease, ledStrip);
  }

  public Score(String location, Arm arm, IntakeRelease intakeRelease, Led2023 ledStrip) {
    if (intakeRelease.getWants() == Wants.CONE) {
      add("Cone", location, arm, intakeRelease, ledStrip);
    } else {
      add("Cube", location, arm, intakeRelease, ledStrip);
    }
  }

  private void add(
      String gamePieceType,
      String location,
      Arm arm,
      IntakeRelease intakeRelease,
      Led2023 ledStrip) {
    addCommands(
        pieceType(gamePieceType, intakeRelease, ledStrip),
        armLocationCommand(location, arm, intakeRelease, ledStrip),
        new ReleaseCMD(intakeRelease, arm));
  }

  private Command armLocationCommand(
      String location, Arm arm, IntakeRelease intakeRelease, Led2023 ledStrip) {
    if (location.equalsIgnoreCase("high")) {
      return new ArmScoreHighNodeCMD(arm, intakeRelease);
    } else if (location.equalsIgnoreCase("mid")) {
      return new ArmScoreMidNodeCMD(arm, intakeRelease);
    } else {
      return new ArmScoreLowNodeCMD(arm, intakeRelease);
    }
  }

  private Command pieceType(String gamePiece, IntakeRelease intakeRelease, Led2023 ledStrip) {
    if (gamePiece.equalsIgnoreCase("cone")) {
      return new WantConeCMD(intakeRelease, ledStrip);
    } else {
      return new WantCubeCMD(intakeRelease, ledStrip);
    }
  }
}
