package frc.robot.commands.arm;

import frc.robot.subsystems.arm.Arm;

public class ArmScoreHighNodeCMD extends ArmPositionCMD {

  public ArmScoreHighNodeCMD(Arm arm) {
    super(arm, 0.3, 0.2);
  }
}
