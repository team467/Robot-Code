package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intakerelease.IntakeAndRaise;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPositionConstants;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.led.Led2023;
import frc.robot.subsystems.led.Led2023.ColorScheme;

public class ArmFloorCMD extends SequentialCommandGroup {

  public ArmFloorCMD(Arm arm, IntakeRelease intakeRelease, Led2023 ledStrip) {
    addCommands(
        new ArmPositionCMD(arm, ArmPositionConstants.FLOOR, ledStrip),
        new IntakeAndRaise(arm, intakeRelease, ledStrip),
        Commands.runOnce(() -> ledStrip.setCmdColorScheme(ColorScheme.FLOOR)));
  }
}
