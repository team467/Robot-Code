package first.robot.opmodes.auto;

import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Trigger;
import org.wpilib.driverstation.RobotState;
import org.wpilib.opmode.OpMode;

public class CommandOpMode implements OpMode {
  private final Trigger enabled = new Trigger(RobotState::isEnabled);

  public CommandOpMode(Supplier<Command> command) {
    enabled.onTrue(command.get());
  }
}
