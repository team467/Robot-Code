package frc.robot.commands.arm;

import frc.robot.subsystems.arm.Arm;

public class ArmScoreLowNodeCMD extends ArmPositionCMD {

  public ArmScoreLowNodeCMD(Arm arm) {
    super(arm, 0.22, 0.065);
  }
}
