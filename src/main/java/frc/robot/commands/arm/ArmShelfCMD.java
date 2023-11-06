package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.effector.IntakeCMD;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositionConstants;
import frc.robot.subsystems.effector.Effector;

public class ArmShelfCMD extends SequentialCommandGroup {

  public ArmShelfCMD(Arm arm, Effector effector) {
    addCommands(
        new ArmPositionCMD(
            arm,
            () ->
                effector.wantsCone()
                    ? ArmPositionConstants.SHELF_CONE_RETRACT
                    : ArmPositionConstants.SHELF_CUBE_RETRACT),
        Commands.parallel(
            new ArmPositionCMD(
                arm,
                () ->
                    effector.wantsCone()
                        ? ArmPositionConstants.SHELF_CONE
                        : ArmPositionConstants.SHELF_CUBE),
            new IntakeCMD(effector)),
        new RaiseArm(arm).unless(() -> !arm.shouldRaise()));
  }
}
