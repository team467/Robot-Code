package frc.robot.commands.arm;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositionConstants;

public class ArmShelfCMD extends ArmPositionCMD {

  public ArmShelfCMD(Arm arm) {

    super(arm, ArmPositionConstants.SHELF);
  }
}
