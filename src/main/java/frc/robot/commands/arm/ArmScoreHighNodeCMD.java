package frc.robot.commands.arm;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositionConstants;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.intakerelease.IntakeRelease.Wants;
import frc.robot.subsystems.led.Led2023;
import frc.robot.subsystems.led.Led2023.ColorScheme;

public class ArmScoreHighNodeCMD extends ArmPositionCMD {

  public ArmScoreHighNodeCMD(Arm arm, IntakeRelease intakerelease, Led2023 ledStrip) {

    super(
        arm,
        () ->
            intakerelease.wantsCone()
                ? ArmPositionConstants.HIGH_CONE
                : ArmPositionConstants.HIGH_CUBE,
        ledStrip);
    if (intakerelease.getWants() == Wants.CUBE
        || (intakerelease.haveCube() && !intakerelease.haveCone())) {
      ledStrip.setCmdColorScheme(ColorScheme.CUBE_HIGH);
    } else {
      ledStrip.setCmdColorScheme(ColorScheme.CONE_HIGH);
    }
  }
}
