package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositionConstants;
import frc.robot.subsystems.intakerelease.IntakeRelease;

public class ArmHomeCMD extends SequentialCommandGroup {

  public ArmHomeCMD(Arm arm, IntakeRelease intakeRelease) {
    addCommands(
        new ArmPositionCMD(
                arm,
                intakeRelease.wantsCone()
                    ? ArmPositionConstants.CONE_HOME
                    : ArmPositionConstants.CUBE_HOME)
            .withTimeout(5.0));
  }
}
