package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.effector.IntakeCMD;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositionConstants;
import frc.robot.subsystems.effector.Effector;

public class ArmFloorCMD extends SequentialCommandGroup {

  public ArmFloorCMD(Arm arm, Effector effector) {
    addCommands(
        new ArmPositionCMD(
            arm,
            () ->
                effector.wantsCone()
                    ? ArmPositionConstants.CONE_FLOOR_RETRACT
                    : ArmPositionConstants.CUBE_FLOOR_RETRACT),
        Commands.parallel(
            new ArmPositionCMD(
                arm,
                () ->
                    effector.wantsCone()
                        ? ArmPositionConstants.CONE_FLOOR
                        : ArmPositionConstants.CUBE_FLOOR),
            new IntakeCMD(effector)),
        new RaiseArm(arm).unless(() -> !arm.shouldRaise()));
  }
}
