package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import java.awt.*;

public class Event {
  private boolean state = false;
  private double condition;
  private double distance;
  private Command executable;

  public Event(double condition, double distance, Command executable) {
    this.condition = condition;
    this.distance = distance;
    this.executable = executable;
  }

  public void setState(boolean state) {
    this.state = state;
  }

  public void setCondition(double condition) {
    this.condition = condition;
  }

  public void checkTrigger(double distance) {
    if (distance <= condition && !state) {
      state = true;
      executable.schedule();
    }
  }
}
