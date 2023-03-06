package frc.robot.commands.auto.complex;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.BackUp;
import frc.robot.commands.auto.Initialize;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.led.Led2023;

public class OnlyBackup extends SequentialCommandGroup {

  public OnlyBackup(
      int aprilTag,
      String relativePosition,
      Drive drive,
      Arm arm,
      IntakeRelease intakeRelease,
      Led2023 ledStrip) {
    addCommands(
        new Initialize(aprilTag, relativePosition, drive, arm, ledStrip),
        new BackUp(drive, arm, ledStrip));
  }
}
