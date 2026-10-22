package first.robot.opmodes.auto;

import com.pathplanner.lib.command3.AutoBuilder;
import org.wpilib.command3.Trigger;
import org.wpilib.driverstation.RobotState;
import org.wpilib.opmode.OpMode;

public class PathPlannerAutoOpMode implements OpMode {
  private final Trigger enabled = new Trigger(RobotState::isEnabled);

  public PathPlannerAutoOpMode(String autoName) {
    enabled.onTrue(AutoBuilder.buildAuto(autoName));
  }
}
