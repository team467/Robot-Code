package frc.robot.commands.arm;

import frc.lib.leds.LEDStrip;
import frc.robot.subsystems.arm.Arm;

public class ArmScoreLowNodeCMD extends ArmPositionCMD {

  public ArmScoreLowNodeCMD(Arm arm) {
    super(arm, 0.239, 0.051);
  }
}
