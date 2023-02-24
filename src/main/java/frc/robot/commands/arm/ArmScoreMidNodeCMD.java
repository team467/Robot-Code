package frc.robot.commands.arm;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositionConstants;
import frc.robot.subsystems.intakerelease.IntakeRelease;

public class ArmScoreMidNodeCMD extends ArmPositionCMD {

  public ArmScoreMidNodeCMD(Arm arm, IntakeRelease intakerelease) {
    super(
        arm,
        intakerelease.haveCone() ? ArmPositionConstants.MID_CONE : ArmPositionConstants.MID_CUBE);
  }
}
