package first.robot.opmodes.teleop;

import first.robot.Robot;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;

@Teleop(name = "Manual", description = "2026 controls with manual intake-arm control")
public class ManualTeleop implements OpMode {
  public ManualTeleop(Robot robot) {
    TeleopBindings.bindCommon(robot);
    TeleopBindings.bindManualMode(robot);
  }
}
