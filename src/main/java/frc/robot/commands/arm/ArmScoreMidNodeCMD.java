package frc.robot.commands.arm;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositionConstants;
import java.util.function.Supplier;

public class ArmScoreMidNodeCMD extends ArmPositionCMD {

  public ArmScoreMidNodeCMD(Arm arm, Supplier<Boolean> wantsCone) {
    super(
        arm, () -> wantsCone.get() ? ArmPositionConstants.MID_CONE : ArmPositionConstants.MID_CUBE);
  }
}
