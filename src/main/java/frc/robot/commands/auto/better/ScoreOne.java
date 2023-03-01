package frc.robot.commands.auto.better;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmCalibrateCMD;
import frc.robot.commands.auto.ScoreConeHigh;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.led.Led2023;

public class ScoreOne extends SequentialCommandGroup {
  public ScoreOne(Drive drive, Arm arm, IntakeRelease intakeRelease, Led2023 ledStrip) {
    addCommands(new ArmCalibrateCMD(arm));
    addCommands(new ScoreConeHigh(drive, arm, intakeRelease, ledStrip, 6));
  }
}
