package frc.robot.commands.arm;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositionConstants;
import frc.robot.subsystems.intakerelease.IntakeRelease;

public class ArmScoreLowNodeCMD extends ArmPositionCMD {

  public ArmScoreLowNodeCMD(Arm arm, IntakeRelease intakerelease) {
    super(arm, ArmPositionConstants.LOW_BOTH);
  }
}
