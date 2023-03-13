package frc.robot.commands.intakerelease;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.RaiseArm;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.led.Led2023;

public class IntakeAndRaise extends SequentialCommandGroup {

  public IntakeAndRaise(Arm arm, IntakeRelease intakerelease, Led2023 ledStrip) {
    addCommands(
        new IntakeCMD(intakerelease, ledStrip), new RaiseArm(arm).unless(() -> !arm.shouldRaise()));
  }
}
