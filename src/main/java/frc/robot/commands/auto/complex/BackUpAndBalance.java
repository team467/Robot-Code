package frc.robot.commands.auto.complex;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.Balancing;
import frc.robot.commands.auto.Initialize;
import frc.robot.commands.auto.StraightDriveToPose;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.Led2023;

public class BackUpAndBalance extends SequentialCommandGroup {
  public BackUpAndBalance(
      String relativePosition,
      Drive drive,
      Arm arm,
      Led2023 ledStrip) {
    int aprilTag = 7;
    addCommands(
        new Initialize(aprilTag, relativePosition, drive, arm, ledStrip),
        new StraightDriveToPose(Units.inchesToMeters(170.0), 0.0, 0.0, drive),
        new StraightDriveToPose(Units.inchesToMeters(-75.0), 0.0, 0.0, drive),
        new Balancing(drive));
  }
}
