package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositionConstants;
import java.util.function.Supplier;

public class ArmHomeCMD extends SequentialCommandGroup {

  public ArmHomeCMD(Arm arm, Supplier<Boolean> wantsCone) {
    addCommands(
        new ArmPositionCMD(
                arm,
                wantsCone.get() ? ArmPositionConstants.CONE_HOME : ArmPositionConstants.CUBE_HOME)
            .withTimeout(5.0));
  }
}
