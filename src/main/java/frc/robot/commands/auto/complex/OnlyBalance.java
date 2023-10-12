package frc.robot.commands.auto.complex;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.commands.auto.BetterBalancing;
import frc.robot.commands.auto.Initialize;
import frc.robot.commands.auto.StraightDriveToPose;
// import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;

public class OnlyBalance extends SequentialCommandGroup {

  public OnlyBalance(String relativePosition, Drive drive) {
    int aprilTag = 7;
    addCommands(
        new Initialize(aprilTag, relativePosition, drive),
        new StraightDriveToPose(0.0, -FieldConstants.Grids.nodeSeparationY, 0.0, drive),
        new StraightDriveToPose(Units.inchesToMeters(95.25), 0.0, 0.0, drive),
        new BetterBalancing(drive));
  }
}
