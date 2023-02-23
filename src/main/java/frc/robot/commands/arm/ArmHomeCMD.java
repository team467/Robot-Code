package frc.robot.commands.arm;

import frc.robot.subsystems.arm.Arm;

public class ArmHomeCMD extends ArmPositionCMD {

  public ArmHomeCMD(Arm arm) {
    super(arm, 0, 0);
  }
}
