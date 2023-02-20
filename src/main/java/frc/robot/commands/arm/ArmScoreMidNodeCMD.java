package frc.robot.commands.arm;

import frc.robot.subsystems.arm.Arm;

public class ArmScoreMidNodeCMD extends ArmPositionCMD {

  public ArmScoreMidNodeCMD(Arm arm) {
    super(arm, 0.2, 0.1);
  }
}
