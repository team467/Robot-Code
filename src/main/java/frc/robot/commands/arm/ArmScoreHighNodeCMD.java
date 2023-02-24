package frc.robot.commands.arm;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositionConstants;
import frc.robot.subsystems.intakerelease.IntakeRelease;

public class ArmScoreHighNodeCMD extends ArmPositionCMD {

  public ArmScoreHighNodeCMD(Arm arm, IntakeRelease intakerelease) {

    super(
        arm,
        intakerelease.haveCone() ? ArmPositionConstants.HIGH_CONE : ArmPositionConstants.HIGH_CUBE);
  }
}
