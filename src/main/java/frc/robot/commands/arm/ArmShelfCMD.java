package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intakerelease.IntakeAndRaise;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositionConstants;
import frc.robot.subsystems.intakerelease.IntakeRelease;

public class ArmShelfCMD extends SequentialCommandGroup {

  public ArmShelfCMD(Arm arm, IntakeRelease intakeRelease) {
    addCommands(
        new ArmPositionCMD(arm, ArmPositionConstants.SHELF),
        new IntakeAndRaise(arm, intakeRelease));
  }
}
