package frc.robot.commands.effector;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.RaiseArm;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.effector.Effector;

public class IntakeAndRaise extends SequentialCommandGroup {

  public IntakeAndRaise(Arm arm, Effector effector) {
    addCommands(new IntakeCMD(effector), new RaiseArm(arm).unless(() -> !arm.shouldRaise()));
  }
}
