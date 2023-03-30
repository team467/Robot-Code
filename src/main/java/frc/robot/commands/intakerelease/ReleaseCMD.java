package frc.robot.commands.intakerelease;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmDropCMD;
import frc.robot.commands.arm.ArmHomeCMD;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intakerelease.IntakeRelease;

public class ReleaseCMD extends SequentialCommandGroup {
  public ReleaseCMD(IntakeRelease intakerelease, Arm arm) {
    if (intakerelease.wantsCone()) {
      addCommands(
          new ArmDropCMD(intakerelease::haveCone, intakerelease::wantsCone, arm).withTimeout(0.3),
          Commands.parallel(
              Commands.run(intakerelease::release, intakerelease).withTimeout(0.5),
              new ArmHomeCMD(arm)).withTimeout(5.0)
      );
    } else {
      addCommands(
          new ArmDropCMD(intakerelease::haveCone, intakerelease::wantsCone, arm).withTimeout(0.3),
          Commands.run(intakerelease::release, intakerelease).withTimeout(0.5),
          new ArmHomeCMD(arm).withTimeout(5.0)
      );
    }
  }
}
