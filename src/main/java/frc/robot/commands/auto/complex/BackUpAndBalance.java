package frc.robot.commands.auto.complex;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.commands.auto.BetterBalancing;
import frc.robot.commands.auto.Initialize;
import frc.robot.commands.auto.StraightDriveToPose;
// import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;

public class BackUpAndBalance extends SequentialCommandGroup {
  public BackUpAndBalance(String relativePosition, Drive drive) {
    int aprilTag = 7;
    addCommands(
        new Initialize(aprilTag, relativePosition, drive),
        new StraightDriveToPose(0.0, -FieldConstants.Grids.nodeSeparationY, 0.0, drive),
        new StraightDriveToPose(Units.inchesToMeters(175.0), 0.0, 0.0, drive),
        new StraightDriveToPose(Units.inchesToMeters(-75.0), 0.0, 0.0, drive).withTimeout(2.5),
        new BetterBalancing(drive));
  }
}