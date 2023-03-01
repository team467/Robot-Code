package frc.robot.commands.arm;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositionConstants;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.intakerelease.IntakeRelease.Wants;
import frc.robot.subsystems.led.Led2023;
import frc.robot.subsystems.led.Led2023.ColorScheme;

public class ArmScoreLowNodeCMD extends ArmPositionCMD {

  public ArmScoreLowNodeCMD(Arm arm, IntakeRelease intakerelease, Led2023 ledStrip) {
    super(arm, ArmPositionConstants.LOW_BOTH, ledStrip);

    if (intakerelease.getWants() == Wants.CUBE) {
      ledStrip.setCmdColorScheme(ColorScheme.CUBE_LOW);
    } else {
      ledStrip.setCmdColorScheme(ColorScheme.CONE_LOW);
    }
    
  }
}
