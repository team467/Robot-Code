package frc.robot.commands.arm;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositionConstants;

public class ArmHomeCMD extends ArmPositionCMD {

  public ArmHomeCMD(Arm arm) {
    super(arm, ArmPositionConstants.HOME);
  }
}
