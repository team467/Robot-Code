package frc.robot.commands.auto.complex;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.Initialize;
import frc.robot.commands.auto.StraightDriveToPose;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intakerelease.IntakeRelease;

public class OnlyBackup extends SequentialCommandGroup {

  public OnlyBackup(
      int aprilTag, String relativePosition, Drive drive, Arm arm, IntakeRelease intakeRelease) {
    addCommands(
        new Initialize(aprilTag, relativePosition, drive, arm),
        new StraightDriveToPose(Units.inchesToMeters(150.0), 0.0, 0.0, drive));
  }
}
