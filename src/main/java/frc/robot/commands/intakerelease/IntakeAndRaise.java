package frc.robot.commands.intakerelease;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.RaiseArm;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intakerelease.IntakeRelease;

public class IntakeAndRaise extends SequentialCommandGroup {

  public IntakeAndRaise(Arm arm, IntakeRelease intakerelease) {
    addCommands(
        new IntakeCMD(intakerelease), new RaiseArm(arm).unless(() -> !arm.shouldRaise()));
  }
}
