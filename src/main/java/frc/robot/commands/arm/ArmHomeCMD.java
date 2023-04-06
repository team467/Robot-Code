package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositionConstants;

public class ArmHomeCMD extends SequentialCommandGroup {

  public ArmHomeCMD(Arm arm) {
    addCommands(new ArmPositionCMD(arm, ArmPositionConstants.HOME).withTimeout(5.0));
  }
}
