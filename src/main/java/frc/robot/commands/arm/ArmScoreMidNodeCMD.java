package frc.robot.commands.arm;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositionConstants;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.intakerelease.IntakeRelease.Wants;
import frc.robot.subsystems.led.Led2023;
import frc.robot.subsystems.led.Led2023.COLORS_467;

public class ArmScoreMidNodeCMD extends ArmPositionCMD {

  public ArmScoreMidNodeCMD(Arm arm, IntakeRelease intakerelease, Led2023 ledStrip) {
    super(
        arm,
        () ->
            intakerelease.haveCone()
                ? ArmPositionConstants.MID_CONE
                : ArmPositionConstants.MID_CUBE,
        ledStrip);
    if (intakerelease.getWants() == Wants.CUBE
        || (intakerelease.haveCube() && !intakerelease.haveCone())) {
      ledStrip.setOneThird(COLORS_467.Purple, 2);
    } else {
      ledStrip.setOneThird(COLORS_467.Yellow, 2);
    }
  }
}
