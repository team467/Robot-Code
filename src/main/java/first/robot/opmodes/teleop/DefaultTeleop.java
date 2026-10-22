package first.robot.opmodes.teleop;

import first.robot.Robot;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;

@Teleop
public class DefaultTeleop implements OpMode {
  public DefaultTeleop(Robot robot) {
    TeleopBindings.bindCommon(robot);
    TeleopBindings.bindAutoMode(robot);
  }
}
