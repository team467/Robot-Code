package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmCalibrateCMD;
import frc.robot.commands.arm.ArmScoreHighNodeCMD;
import frc.robot.commands.intakerelease.ReleaseCMD;
import frc.robot.commands.intakerelease.WantCubeCMD;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.led.Led2023;

public class ScoreCubeNoMove extends SequentialCommandGroup {
  public ScoreCubeNoMove(Arm arm, IntakeRelease intakeRelease, Led2023 ledStrip) {
    addCommands(
        new ArmCalibrateCMD(arm),
        new WantCubeCMD(intakeRelease, ledStrip),
        new ArmScoreHighNodeCMD(arm, intakeRelease));
    new ReleaseCMD(intakeRelease, ledStrip);
  }
}
