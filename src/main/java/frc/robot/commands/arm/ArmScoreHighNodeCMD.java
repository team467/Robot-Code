package frc.robot.commands.arm;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositionConstants;
import java.util.function.Supplier;

public class ArmScoreHighNodeCMD extends ArmPositionCMD {

  public ArmScoreHighNodeCMD(Arm arm, Supplier<Boolean> wantsCone) {

    super(
        arm,
        () -> wantsCone.get() ? ArmPositionConstants.HIGH_CONE : ArmPositionConstants.HIGH_CUBE);
  }
}
