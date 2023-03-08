package frc.robot.commands.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.arm.ArmHomeCMD;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.Led2023;

public class BackUp extends ParallelCommandGroup {
  public BackUp(Drive drive, Arm arm, Led2023 ledStrip) {
    addCommands(
        new StraightDriveToPose(Units.inchesToMeters(150.0), 0.0, 0.0, drive),
        new ArmHomeCMD(arm, ledStrip));
  }
}
