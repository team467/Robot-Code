package frc.robot.opmodes;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.OpMode;

public class AutoOpMode implements OpMode {

  Command command;

  public AutoOpMode(Command command) {
    this.command = command;
  }

  @Override
  public void start() {
    Scheduler.getDefault().schedule(command);
  }
}
