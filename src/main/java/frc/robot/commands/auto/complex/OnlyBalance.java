package frc.robot.commands.auto.complex;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.BetterBalancing;
import frc.robot.commands.auto.Initialize;
import frc.robot.commands.auto.StraightDriveToPose;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;

public class OnlyBalance extends SequentialCommandGroup {

  public OnlyBalance(String relativePosition, Drive drive, Arm arm) {
    int aprilTag = 7;
    addCommands(
        new Initialize(aprilTag, relativePosition, drive, arm),
        new StraightDriveToPose(Units.inchesToMeters(95.25), 0.0, 0.0, drive),
        new BetterBalancing(drive));
  }
}
