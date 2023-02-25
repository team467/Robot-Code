package frc.robot.commands.arm;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositionConstants;

public class ArmFloorCMD extends ArmPositionCMD {

  public ArmFloorCMD(Arm arm) {
    super(arm, ArmPositionConstants.FLOOR);
  }
}
